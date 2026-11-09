#!/usr/bin/env python3
"""Log PX4 vehicle position (north, east, altitude) from /fmu/out/vehicle_odometry."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleOdometry


class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile_sensor_data,
        )

    def odometry_callback(self, msg: VehicleOdometry):
        north, east, down = (float(v) for v in msg.position)
        if any(math.isnan(v) for v in (north, east, down)):
            self.get_logger().warn('Position not valid (NaN)', throttle_duration_sec=0.5)
            return
        self.get_logger().info(
            f'N: {north:8.2f} m  E: {east:8.2f} m  Alt: {-down:8.2f} m',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
