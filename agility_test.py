import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry

NAN = float('nan')

class AgilityTest(Node):
    def __init__(self):
        super().__init__('agility_test')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.pos = [0.0, 0.0, 0.0]; self.vel = [0.0, 0.0, 0.0]
        self.ticks = 0; self.phase = 'takeoff'; self.t0 = 0
        self.max_speed = 0.0; self.brake_start = None; self.brake_time = None
        self.create_timer(0.05, self.loop)  # 20 Hz

    def odom_cb(self, m):
        self.pos = list(m.position); self.vel = list(m.velocity)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(command=command, param1=p1, param2=p2, target_system=1,
                           target_component=1, source_system=1, source_component=1,
                           from_external=True, timestamp=self.now())
        self.cmd_pub.publish(m)

    def now(self):
        return self.get_clock().now().nanoseconds // 1000

    def send(self, pos=None, vel=None):
        mode = OffboardControlMode(timestamp=self.now())
        sp = TrajectorySetpoint(timestamp=self.now(), yaw=NAN)
        if vel is None:
            mode.position = True
            sp.position = pos; sp.velocity = [NAN] * 3
        else:
            mode.velocity = True
            sp.position = [NAN, NAN, NAN] if pos is None else pos
            sp.velocity = vel
        self.mode_pub.publish(mode); self.sp_pub.publish(sp)

    def loop(self):
        self.ticks += 1
        t = (self.ticks - self.t0) * 0.05
        speed = math.hypot(self.vel[0], self.vel[1])
        self.max_speed = max(self.max_speed, speed)

        if 10 <= self.ticks <= 100:
            if self.ticks % 10 == 0: self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.ticks % 10 == 5: self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.phase == 'takeoff':
            self.send(pos=[0.0, 0.0, -10.0])
            if self.ticks > 100 and abs(self.pos[2] + 10) < 0.5:
                self.next('sprint')
        elif self.phase == 'sprint':
            self.send(vel=[20.0, 0.0, 0.0], pos=[NAN, NAN, -10.0])
            if t > 4: self.brake_start = self.ticks; self.next('brake')
        elif self.phase == 'brake':
            self.send(vel=[0.0, 0.0, 0.0], pos=[NAN, NAN, -10.0])
            if self.brake_time is None and speed < 0.5:
                self.brake_time = (self.ticks - self.brake_start) * 0.05
            if t > 4: self.next('slalom')
        elif self.phase == 'slalom':
            east = 8.0 * math.sin(2 * math.pi * t / 2.5)
            self.send(vel=[-12.0, east, 0.0], pos=[NAN, NAN, -10.0])
            if t > 8: self.next('home')
        elif self.phase == 'home':
            self.send(pos=[0.0, 0.0, -10.0])
            if math.hypot(self.pos[0], self.pos[1]) < 1.0 and t > 3:
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND); self.next('land')
        elif self.phase == 'land':
            if t > 1 and self.pos[2] > -0.3:
                self.get_logger().info(f'DONE. Top speed: {self.max_speed:.1f} m/s, '
                                       f'braking 20->0 took: {self.brake_time} s')
                raise SystemExit

        if self.ticks % 10 == 0:
            self.get_logger().info(f'{self.phase}: speed {speed:.1f} m/s, alt {-self.pos[2]:.1f} m')

    def next(self, phase):
        self.get_logger().info(f'--> {phase}')
        self.phase = phase; self.t0 = self.ticks

def main():
    rclpy.init(); node = AgilityTest()
    try: rclpy.spin(node)
    except SystemExit: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()