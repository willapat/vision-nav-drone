import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry

# North, East, Down in meters (down is negative = up)
WAYPOINTS = [(0, 0, -5), (5, 0, -5), (5, 5, -5), (0, 5, -5), (0, 0, -5)]
REACHED_M = 0.5


class OffboardSquare(Node):
    def __init__(self):
        super().__init__('offboard_square')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odom, qos_profile_sensor_data)

        self.position = None
        self.ticks = 0
        self.index = 0
        self.landing = False
        self.create_timer(0.1, self.loop)  # 10 Hz; PX4 needs > 2 Hz to stay in offboard

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def on_odom(self, msg):
        self.position = msg.position

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1, msg.param2 = p1, p2
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.cmd_pub.publish(msg)

    def loop(self):
        if self.landing:
            return

        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = self.now_us()
        self.mode_pub.publish(mode)

        target = WAYPOINTS[self.index]
        sp = TrajectorySetpoint()
        sp.position = [float(v) for v in target]
        sp.yaw = 0.0
        sp.timestamp = self.now_us()
        self.setpoint_pub.publish(sp)

        # Stream setpoints for 1 s first, then switch to offboard and arm
        if 10 <= self.ticks <= 100:
            if self.ticks % 10 == 0:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif self.ticks % 10 == 5:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info('Offboard + arm sent')
        self.ticks += 1

        if self.position is None or self.ticks < 10:
            return
        dist = sum((a - b) ** 2 for a, b in zip(self.position, target)) ** 0.5
        if dist < REACHED_M:
            self.get_logger().info(f'Reached waypoint {self.index}: {target}')
            self.index += 1
            if self.index == len(WAYPOINTS):
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().info('Square done, landing')
                self.landing = True


def main():
    rclpy.init()
    rclpy.spin(OffboardSquare())


if __name__ == '__main__':
    main()
