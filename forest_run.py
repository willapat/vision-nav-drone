"""Fly home -> through the forest -> goal past the tree line -> home, as fast as is safe.

Pipeline (known tree positions from the world file, a camera comes later):
  1. Obstacles: every tree is a circle whose radius is the widest part of its collision
     mesh inside the height band the drone occupies at ALT (so canopies count when you
     fly high), plus the drone's own radius.
  2. Clearance grid (GRID_RES) over the area, A* on it with a cost that prefers open space.
  3. Smoothing: line-of-sight shortcutting that never lowers clearance, then elastic
     corner rounding.
  4. Speed profile: MAX_SPEED, capped by curvature (lateral accel) and clearance, then
     accel/decel limited.
  5. Follower: position setpoint on the nearest path point (cross-track correction) with
     velocity + acceleration feed-forward taken from a lookahead point on the path.
"""
import heapq, math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry

MAX_SPEED = 6.0      # m/s, raise this to push it
ALT = 0.9            # m above the takeoff point. Oak canopies start ~1.4 m above ground,
                     # so up to ~0.95 the drone flies under them. Higher works too, the
                     # planner just sees fatter trees (and may find no way through).
MAX_ACC = 4.0        # m/s^2 along the path (speed up / brake)
MAX_LAT_ACC = 3.0    # m/s^2 in turns (combined with MAX_ACC as a friction circle)
K_XTRACK = 1.0       # 1/s, cross-track correction added on top of PX4's own position gain
ACC_FF = 0.5         # 0..1, acceleration feed-forward gain
DRONE_R = 0.40       # m, x500 centre to prop tip
MARGIN = 0.35        # m, hard minimum gap between prop tip and tree (tracking + EKF error)
COMFORT = 1.5        # m, gap at which full speed is allowed; below it the drone slows down
V_TIGHT = 1.5        # m/s, speed in the tightest allowed gaps
GRID_RES = 0.2       # m
HOVER_AT_GOAL = 1.0  # s
NAN = float('nan')
WORLD = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf')

# Widest radius (m) of each tree's collision mesh in 0.1 m height slices from the ground up,
# measured from the Gazebo Fuel meshes (oak tree v7, pine tree v6).
PROFILE = {
    'Oak': [1.90, 1.70, 1.29, 1.01, 0.80, 0.63, 0.54, 0.50, 0.47, 0.47, 0.47, 0.49, 0.51, 0.53,
            2.89, 3.86, 3.74, 3.63, 3.68, 3.79, 3.78, 3.79, 3.83, 3.98, 4.03, 4.13, 4.09, 4.46,
            4.61, 5.02, 5.25, 5.29, 5.38, 5.63, 5.87, 5.95, 5.80, 5.69, 5.72, 5.72, 5.78, 5.85,
            4.66, 4.69, 4.75, 4.07, 4.18, 4.33, 4.35, 4.34, 4.40, 4.21, 4.19, 4.21, 4.23, 4.25,
            4.30, 4.30, 4.10, 3.82, 3.78, 3.83, 3.47, 3.38, 2.07, 1.46],
    'Pine': [0.15, 0.10, 0.09, 0.08, 0.08, 0.08, 1.59, 1.59, 1.59, 1.55, 1.58, 1.59, 1.49, 1.50,
             1.43, 1.47, 1.46, 1.32, 1.21, 1.19, 1.23, 1.20, 1.21, 1.13, 1.00, 0.91, 0.84, 0.86,
             0.85, 0.84, 0.84, 0.85, 0.79, 0.68, 0.69, 0.67, 0.67, 0.59, 0.56, 0.56, 0.50, 0.41,
             0.32, 0.32, 0.30, 0.24, 0.17, 0.13, 0.10, 0.06, 0.03],
}
# Height band (m above ground) the x500 occupies: base_link sits ALT above ground at hover
# height, landing gear reaches ~0.23 below it and the props ~0.07 above; +-0.15 tracking slop.
BAND = (-0.15, 0.45)


def tree_radius(kind, lo, hi):
    profiles = [PROFILE[kind]] if kind in PROFILE else list(PROFILE.values())
    r = 0.0
    for p in profiles:
        i0, i1 = max(int(lo / 0.1), 0), int(hi / 0.1)
        r = max([r] + p[i0:i1 + 1])
    return r


def load_trees():
    """[(north, east, kind)] in PX4 NED from the Gazebo ENU world file."""
    trees, kind = [], None
    for line in open(WORLD):
        if '<uri>' in line:
            kind = next((k for k in PROFILE if f'{k} Tree' in line), 'Tree' if 'Tree' in line else None)
        elif '<pose>' in line and kind:
            x, y = map(float, line.split('<pose>')[1].split()[:2])
            trees.append((y, x, kind))   # ENU (x east, y north) -> NED (north, east)
            kind = None
    return trees


class Planner:
    def __init__(self, trees, alt, extra_pts):
        self.tn = np.array([t[0] for t in trees])
        self.te = np.array([t[1] for t in trees])
        self.tr = np.array([tree_radius(t[2], alt + BAND[0], alt + BAND[1]) for t in trees]) + DRONE_R
        pts = np.array([[t[0], t[1]] for t in trees] + list(extra_pts))
        self.lo = pts.min(0) - 8.0
        hi = pts.max(0) + 8.0
        self.shape = tuple(np.ceil((hi - self.lo) / GRID_RES).astype(int) + 1)
        n = self.lo[0] + GRID_RES * np.arange(self.shape[0])
        e = self.lo[1] + GRID_RES * np.arange(self.shape[1])
        N, E = np.meshgrid(n, e, indexing='ij')
        self.clr_grid = self.clearance(N, E)

    def clearance(self, n, e):
        """Gap between prop tip and the nearest tree surface at flight height (m)."""
        n, e = np.asarray(n, float), np.asarray(e, float)
        d = np.hypot(n[..., None] - self.tn, e[..., None] - self.te) - self.tr
        return d.min(-1)

    def cell(self, p):
        i, j = np.round((np.asarray(p) - self.lo) / GRID_RES).astype(int)
        return int(np.clip(i, 0, self.shape[0] - 1)), int(np.clip(j, 0, self.shape[1] - 1))

    def astar(self, start, goal):
        free = self.clr_grid >= MARGIN
        # step cost multiplier: 1 in open space, up to 5 near the hard limit
        mult = (1.0 + 4.0 * np.clip((COMFORT - self.clr_grid) / (COMFORT - MARGIN), 0, 1)).tolist()
        free = free.tolist()
        s, g = self.cell(start), self.cell(goal)
        free[s[0]][s[1]] = free[g[0]][g[1]] = True
        H, W = self.shape
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])
        best = {s: 0.0}; parent = {s: None}; openq = [(h(s), s)]
        while openq:
            _, c = heapq.heappop(openq)
            if c == g:
                break
            gc = best[c]
            for di, dj, L in moves:
                nb = (c[0] + di, c[1] + dj)
                if not (0 <= nb[0] < H and 0 <= nb[1] < W) or not free[nb[0]][nb[1]]:
                    continue
                ng = gc + L * 0.5 * (mult[c[0]][c[1]] + mult[nb[0]][nb[1]])
                if ng < best.get(nb, 1e18):
                    best[nb] = ng; parent[nb] = c
                    heapq.heappush(openq, (ng + h(nb), nb))
        if g not in parent:
            return None
        path, c = [], g
        while c is not None:
            path.append(self.lo + GRID_RES * np.array(c)); c = parent[c]
        path.reverse()
        path[0], path[-1] = np.array(start, float), np.array(goal, float)
        return np.array(path)

    def seg_clearance(self, a, b):
        k = max(int(np.linalg.norm(b - a) / 0.05), 1)
        t = np.linspace(0, 1, k + 1)[:, None]
        p = a + t * (b - a)
        return self.clearance(p[:, 0], p[:, 1]).min()

    def shortcut(self, path):
        clr = self.clearance(path[:, 0], path[:, 1])
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                need = min(COMFORT, clr[i:j + 1].min()) - 0.02   # never trade away clearance
                if self.seg_clearance(path[i], path[j]) >= need:
                    break
                j -= 1
            out.append(path[j]); i = j
        return np.array(out)

    def round_corners(self, pts, spacing=0.1, iters=300):
        # resample
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.r_[0, np.cumsum(seg)]
        ss = np.linspace(0, s[-1], max(int(s[-1] / spacing), 2) + 1)
        p = np.c_[np.interp(ss, s, pts[:, 0]), np.interp(ss, s, pts[:, 1])]
        clr = self.clearance(p[:, 0], p[:, 1])
        for _ in range(iters):
            q = p.copy()
            q[1:-1] += 0.3 * (p[:-2] + p[2:] - 2 * p[1:-1])
            c = self.clearance(q[:, 0], q[:, 1])
            ok = (c >= np.minimum(clr, COMFORT) - 0.01) & (c >= MARGIN)
            p[ok] = q[ok]; clr[ok] = c[ok]
        return p

    def plan(self, start, goal):
        raw = self.astar(start, goal)
        if raw is None:
            return None
        p = self.round_corners(self.shortcut(raw))
        return Path(p, self.clearance(p[:, 0], p[:, 1]))


class Path:
    def __init__(self, p, clr):
        self.p, self.clr = p, clr
        d = np.diff(p, axis=0)
        self.s = np.r_[0, np.cumsum(np.linalg.norm(d, axis=1))]
        self.length = self.s[-1]
        t = np.gradient(p, self.s, axis=0)
        self.tan = t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        # signed curvature from the heading, smoothed (raw finite differences on a 0.1 m
        # grid are noisy); normal = tangent rotated +90 deg, so acc = v^2 * kappa * normal
        heading = np.unwrap(np.arctan2(self.tan[:, 1], self.tan[:, 0]))
        kappa = np.gradient(heading, self.s)
        kappa = np.convolve(np.pad(kappa, 7, mode='edge'), np.ones(15) / 15, 'valid')
        self.kappa = kappa
        self.normal = np.c_[-self.tan[:, 1], self.tan[:, 0]]
        v = np.full(len(p), MAX_SPEED)
        v = np.minimum(v, np.sqrt(MAX_LAT_ACC / np.maximum(np.abs(kappa), 1e-6)))
        frac = np.clip((clr - MARGIN) / (COMFORT - MARGIN), 0, 1)
        v = np.minimum(v, V_TIGHT + (MAX_SPEED - V_TIGHT) * frac)
        v[0] = v[-1] = 0.0
        ds = np.diff(self.s)
        ak = np.abs(kappa)

        def a_along(i, vi):                   # what is left for speeding up / braking in a turn
            lat = vi * vi * ak[i]
            return MAX_ACC * math.sqrt(max(0.05, 1.0 - (lat / MAX_LAT_ACC) ** 2))
        for i in range(1, len(v)):            # accelerate
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_along(i - 1, v[i - 1]) * ds[i - 1]))
        for i in range(len(v) - 2, -1, -1):   # brake
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_along(i + 1, v[i + 1]) * ds[i]))
        self.v = v
        acc_t = 0.5 * np.gradient(v * v, self.s)         # along-path accel (v dv/ds) for feed-forward
        self.acc_t = np.convolve(np.pad(acc_t, 7, mode='edge'), np.ones(15) / 15, 'valid')
        self.eta = float(np.sum(ds / np.maximum(0.5 * (v[1:] + v[:-1]), 0.3)))

    def at(self, s):
        i = int(np.clip(np.searchsorted(self.s, s), 0, len(self.s) - 1))
        return i


class ForestRun(Node):
    def __init__(self):
        super().__init__('forest_run')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.trees = load_trees()
        far_e = max(t[1] for t in self.trees) + 8.0
        mid_n = sum(t[0] for t in self.trees) / len(self.trees)
        self.goal = np.array([mid_n, far_e])
        self.get_logger().info(f'{len(self.trees)} trees loaded, goal N{mid_n:.1f} E{far_e:.1f}, '
                               f'ALT {ALT} m, MAX_SPEED {MAX_SPEED} m/s')
        self.pos = None; self.vel = [0.0] * 3; self.tilt = 0.0
        self.z0_samples = []; self.z0 = None; self.home = None
        self.ticks = 0; self.phase = 'wait'
        self.planner = None; self.path = None; self.idx = 0; self.legs = []
        self.min_clear = 99.0; self.min_trunk = 99.0; self.max_xtrack = 0.0; self.unsafe_ticks = 0
        self.dist = 0.0; self.last_pos = None; self.t_start = None; self.t_hover = None
        log = os.environ.get('FOREST_CSV')      # FOREST_CSV=run.csv logs setpoints vs state at 20 Hz
        self.csv = open(log, 'w') if log else None
        if self.csv:
            self.csv.write('t,n,e,vn,ve,sp_n,sp_e,ff_vn,ff_ve,ff_an,ff_ae\n')
        self.create_timer(0.05, self.loop)

    # ---------------------------------------------------------------- io
    def odom_cb(self, m):
        self.pos = list(m.position); self.vel = list(m.velocity)
        w, x, y, z = m.q
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def now(self):
        return self.get_clock().now().nanoseconds // 1000

    def cmd(self, command, p1=0.0, p2=0.0):
        self.cmd_pub.publish(VehicleCommand(command=command, param1=p1, param2=p2,
            target_system=1, target_component=1, source_system=1, source_component=1,
            from_external=True, timestamp=self.now()))

    def send(self, pos, vel=(NAN, NAN, NAN), acc=(NAN, NAN, NAN), yaw=NAN):
        self.mode_pub.publish(OffboardControlMode(timestamp=self.now(), position=True,
                                                  velocity=True, acceleration=True))
        self.sp_pub.publish(TrajectorySetpoint(timestamp=self.now(), position=[float(v) for v in pos],
            velocity=[float(v) for v in vel], acceleration=[float(v) for v in acc], yaw=float(yaw)))
        if self.csv:
            self.csv.write(','.join(f'{x:.3f}' for x in [time.monotonic(), *self.pos[:2], *self.vel[:2],
                                                         *pos[:2], *vel[:2], *acc[:2]]) + '\n')

    def z_sp(self):
        return self.z0 - ALT

    # ---------------------------------------------------------------- logic
    def start_leg(self, goal, name):
        start = np.array(self.pos[:2])
        t0 = time.monotonic()
        self.path = self.planner.plan(start, goal)
        if self.path is None:
            self.get_logger().error(f'No path to {name} at ALT {ALT} m: trees too wide at this height. '
                                    f'Try a lower ALT or a smaller MARGIN. Holding.')
            self.phase = 'hold'; self.hold_pos = list(self.pos[:2]); return
        p = self.path
        self.get_logger().info(f'--> {name}: path {p.length:.1f} m, min gap {p.clr.min():.2f} m, '
                               f'ETA {p.eta:.1f} s, peak {p.v.max():.1f} m/s '
                               f'(planned in {time.monotonic() - t0:.2f} s)')
        self.idx = 0; self.leg_name = name; self.leg_t0 = time.monotonic(); self.leg_d0 = self.dist
        self.phase = 'fly'

    def follow(self):
        p, pos = self.path, np.array(self.pos[:2])
        hi = min(self.idx + 80, len(p.p))           # search 8 m ahead, never backwards
        k = self.idx + int(np.argmin(np.linalg.norm(p.p[self.idx:hi] - pos, axis=1)))
        self.idx = k
        xtrack = float(np.linalg.norm(p.p[k] - pos))
        to_end = float(np.linalg.norm(p.p[-1] - pos))
        if p.length - p.s[k] < 1.0 or to_end < 1.0:   # final metre: settle on the end point
            g = p.p[-1]
            self.send([g[0], g[1], self.z_sp()], yaw=math.atan2(p.tan[-1][1], p.tan[-1][0]))
            return to_end < 0.3 and math.hypot(self.vel[0], self.vel[1]) < 0.5, xtrack
        self.max_xtrack = max(self.max_xtrack, xtrack)
        v = max(p.v[k], 0.6)
        la = p.at(p.s[k] + 0.2 + 0.15 * v)           # lookahead: anticipate the controller lag
        v = max(p.v[la], 0.6)
        t = p.tan[la]
        acc = ACC_FF * (p.normal[k] * float(np.clip(v * v * p.kappa[k], -MAX_LAT_ACC, MAX_LAT_ACC))
                        + p.tan[k] * float(np.clip(p.acc_t[k], -MAX_ACC, MAX_ACC)))
        yaw = math.atan2(t[1], t[0])
        corr = K_XTRACK * (p.p[k] - pos)               # extra pull back onto the path
        self.send([p.p[k][0], p.p[k][1], self.z_sp()], [v * t[0] + corr[0], v * t[1] + corr[1], 0.0],
                  [acc[0], acc[1], 0.0], yaw)
        return p.length - p.s[k] < 0.3 and float(np.linalg.norm(p.p[-1] - pos)) < 0.5, xtrack

    def track_safety(self):
        n, e = self.pos[0], self.pos[1]
        trunk = float(np.hypot(self.planner.tn - n, self.planner.te - e).min())
        h = self.z0 - self.pos[2]
        tr = np.array([tree_radius(t[2], h + BAND[0] + 0.15, h + BAND[1] - 0.15) for t in self.trees])
        gap = float((np.hypot(self.planner.tn - n, self.planner.te - e) - tr - DRONE_R).min())
        self.min_trunk = min(self.min_trunk, trunk); self.min_clear = min(self.min_clear, gap)
        if gap < 0.0:
            self.unsafe_ticks += 1
            if self.unsafe_ticks % 20 == 1:
                self.get_logger().warn(f'CONTACT RISK: prop tip {gap:.2f} m from a tree at h={h:.2f} m')
        if self.tilt > 50.0:
            self.get_logger().error(f'Tilt {self.tilt:.0f} deg, looks like a crash. Landing.')
            self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND); self.phase = 'land'

    def loop(self):
        self.ticks += 1
        if 10 <= self.ticks <= 100:
            if self.ticks % 10 == 0: self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.ticks % 10 == 5: self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self.pos is None:
            return
        if self.z0 is None:                 # ground reference before anything moves
            self.z0_samples.append(self.pos[2])
            if len(self.z0_samples) < 8:
                return
            self.z0 = float(np.median(self.z0_samples)); self.home = np.array(self.pos[:2])
            self.planner = Planner(self.trees, ALT, [self.home, self.goal])
            self.get_logger().info(f'ground z {self.z0:.2f}, home N{self.home[0]:.2f} E{self.home[1]:.2f}, '
                                   f'obstacle radii at ALT: '
                                   f'{sorted(set(np.round(self.planner.tr - DRONE_R, 2)))} m')
            self.phase = 'takeoff'
        speed = math.hypot(self.vel[0], self.vel[1])
        if self.phase in ('fly', 'hover'):
            pos = np.array(self.pos[:2])
            if self.last_pos is not None:
                self.dist += float(np.linalg.norm(pos - self.last_pos))
            self.last_pos = pos
            self.track_safety()

        if self.phase == 'takeoff':
            self.send([self.home[0], self.home[1], self.z_sp()])
            if self.ticks > 100 and abs(self.pos[2] - self.z_sp()) < 0.15 and abs(self.vel[2]) < 0.3:
                self.t_start = time.monotonic()
                self.start_leg(self.goal, 'goal')
        elif self.phase == 'fly':
            done, xtrack = self.follow()
            if self.ticks % 10 == 0:
                self.get_logger().info(f'{self.leg_name}: {speed:4.1f} m/s, '
                                       f'{self.path.length - self.path.s[self.idx]:5.1f} m to go, '
                                       f'xtrack {xtrack:.2f} m, h {self.z0 - self.pos[2]:.2f} m')
            if done:
                self.legs.append((self.leg_name, time.monotonic() - self.leg_t0, self.dist - self.leg_d0))
                self.get_logger().info(f'Reached {self.leg_name} in {self.legs[-1][1]:.1f} s')
                if self.leg_name == 'goal':
                    self.phase = 'hover'; self.t_hover = time.monotonic()
                else:
                    self.finish()
        elif self.phase == 'hover':
            g = self.path.p[-1]
            self.send([g[0], g[1], self.z_sp()])
            if time.monotonic() - self.t_hover > HOVER_AT_GOAL:
                self.start_leg(self.home, 'home')
        elif self.phase == 'hold':
            self.send([self.hold_pos[0], self.hold_pos[1], self.z_sp()])
        elif self.phase == 'land':
            if self.pos[2] > self.z0 - 0.15 and abs(self.vel[2]) < 0.1:
                self.get_logger().info('Landed.')
                raise SystemExit

    def finish(self):
        total = time.monotonic() - self.t_start
        fly_t = sum(l[1] for l in self.legs)
        self.get_logger().info('=' * 60)
        for name, t, d in self.legs:
            self.get_logger().info(f'  leg to {name:4s}: {t:5.1f} s, {d:5.1f} m, avg {d / t:.2f} m/s')
        self.get_logger().info(f'  total time (takeoff done -> home, incl. {HOVER_AT_GOAL:.0f} s hover): {total:.1f} s')
        self.get_logger().info(f'  average speed while flying: {self.dist / fly_t:.2f} m/s '
                               f'(MAX_SPEED {MAX_SPEED}, ALT {ALT})')
        self.get_logger().info(f'  closest to any trunk (centre to trunk axis): {self.min_trunk:.2f} m')
        self.get_logger().info(f'  closest prop tip to any tree surface: {self.min_clear:.2f} m')
        self.get_logger().info(f'  max cross-track error: {self.max_xtrack:.2f} m, '
                               f'contact-risk ticks: {self.unsafe_ticks}')
        self.get_logger().info('=' * 60)
        self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND); self.phase = 'land'


def main():
    rclpy.init(); node = ForestRun()
    try: rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt): pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
