import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanReader(Node):

    def __init__(self):
        super().__init__('scan_reader')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.obstacle_distance = 0.8

        self.get_logger().info('Scan reader node started.')

    def get_sector_min(self, msg, start_angle_deg, end_angle_deg):
        start_angle_rad = math.radians(start_angle_deg)
        end_angle_rad = math.radians(end_angle_deg)

        start_index = int(
            (start_angle_rad - msg.angle_min)
            / msg.angle_increment
        )

        end_index = int(
            (end_angle_rad - msg.angle_min)
            / msg.angle_increment
        )

        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        sector_ranges = msg.ranges[
            start_index:end_index + 1
        ]

        valid_ranges = [
            distance
            for distance in sector_ranges
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            )
        ]

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)
    
    def scan_callback(self, msg):

        # Front is the wrap-around region near +/-180 degrees.
        front_positive = self.get_sector_min(
            msg,
            160.0,
            180.0
        )

        front_negative = self.get_sector_min(
            msg,
            -180.0,
            -160.0
        )

        front_distance = min(
            front_positive,
            front_negative
        )

        # Left and right already match the expected physical directions.
        left_distance = self.get_sector_min(
            msg,
            60.0,
            120.0
        )

        right_distance = self.get_sector_min(
            msg,
            -120.0,
            -60.0
        )

        front_blocked = (
            front_distance < self.obstacle_distance
        )

        left_blocked = (
            left_distance < self.obstacle_distance
        )

        right_blocked = (
            right_distance < self.obstacle_distance
        )

        self.get_logger().info(
            f'Front: {front_distance:.2f} m '
            f'({"BLOCKED" if front_blocked else "CLEAR"}) | '
            f'Left: {left_distance:.2f} m '
            f'({"BLOCKED" if left_blocked else "CLEAR"}) | '
            f'Right: {right_distance:.2f} m '
            f'({"BLOCKED" if right_blocked else "CLEAR"})'
        )

def main(args=None):
    rclpy.init(args=args)

    node = ScanReader()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()