import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =====================================================
        # DISTANCE THRESHOLDS
        # =====================================================

        self.emergency_distance = 0.35
        self.obstacle_distance = 0.80
        self.clear_distance = 1.10
        self.side_clear_distance = 1.20

        # Require several consecutive good scans before changing state.
        self.clear_scan_required = 5
        self.clear_scan_count = 0

        # =====================================================
        # SPEEDS
        # =====================================================

        self.forward_speed = 0.15
        self.turning_forward_speed = 0.04
        self.passing_forward_speed = 0.10
        self.turn_speed = 0.35

        # =====================================================
        # STATE MACHINE
        # =====================================================

        self.state = 'FORWARD'
        self.turn_direction = None

        self.get_logger().info(
            'Improved obstacle avoidance node started.'
        )


    def get_sector_min(self, scan, start_angle_deg, end_angle_deg):

        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        valid_ranges = []

        for index, distance in enumerate(scan.ranges):

            angle = scan.angle_min + index * scan.angle_increment

            if start_angle <= angle <= end_angle:

                if not math.isfinite(distance):
                    continue

                if distance < scan.range_min:
                    continue

                if distance > scan.range_max:
                    continue

                valid_ranges.append(distance)

        if not valid_ranges:
            return float('inf')

        return min(valid_ranges)


    def stop_robot(self):

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        self.cmd_vel_publisher.publish(command)


    def scan_callback(self, scan):

        # =====================================================
        # SECTORS
        # =====================================================

        # Narrow front sector.
        front = self.get_sector_min(
            scan,
            -15.0,
            15.0
        )

        # Front-left and front-right help detect corners.
        front_left = self.get_sector_min(
            scan,
            15.0,
            60.0
        )

        front_right = self.get_sector_min(
            scan,
            -60.0,
            -15.0
        )

        # Side sectors.
        left = self.get_sector_min(
            scan,
            60.0,
            120.0
        )

        right = self.get_sector_min(
            scan,
            -120.0,
            -60.0
        )

        command = Twist()

        # =====================================================
        # EMERGENCY SAFETY
        # =====================================================

        minimum_nearby = min(
            front,
            front_left,
            front_right
        )

        if minimum_nearby < self.emergency_distance:

            command.linear.x = 0.0

            # Turn away from the closer side.
            if front_left < front_right:
                command.angular.z = -self.turn_speed
                self.turn_direction = 'RIGHT'
            else:
                command.angular.z = self.turn_speed
                self.turn_direction = 'LEFT'

            self.state = 'TURNING'
            self.clear_scan_count = 0

            self.get_logger().warning(
                f'EMERGENCY | '
                f'Front={front:.2f} | '
                f'FL={front_left:.2f} | '
                f'FR={front_right:.2f}'
            )

            self.cmd_vel_publisher.publish(command)
            return


        # =====================================================
        # FORWARD
        # =====================================================

        if self.state == 'FORWARD':

            if front >= self.obstacle_distance:

                command.linear.x = self.forward_speed
                command.angular.z = 0.0

            else:

                # Use both frontal and side information.
                left_score = min(front_left, left)
                right_score = min(front_right, right)

                if left_score > right_score:
                    self.turn_direction = 'LEFT'
                else:
                    self.turn_direction = 'RIGHT'

                self.state = 'TURNING'
                self.clear_scan_count = 0

                command.linear.x = self.turning_forward_speed

                if self.turn_direction == 'LEFT':
                    command.angular.z = self.turn_speed
                else:
                    command.angular.z = -self.turn_speed

                self.get_logger().info(
                    f'OBSTACLE | '
                    f'Front={front:.2f} | '
                    f'FL={front_left:.2f} | '
                    f'FR={front_right:.2f} | '
                    f'Left={left:.2f} | '
                    f'Right={right:.2f} | '
                    f'Turn={self.turn_direction}'
                )


        # =====================================================
        # TURNING
        # =====================================================

        elif self.state == 'TURNING':

            # Re-evaluate whether the currently selected direction
            # is still safe.
            if self.turn_direction == 'LEFT':

                if front_left < self.emergency_distance:
                    self.turn_direction = 'RIGHT'

            else:

                if front_right < self.emergency_distance:
                    self.turn_direction = 'LEFT'


            command.linear.x = self.turning_forward_speed

            if self.turn_direction == 'LEFT':
                command.angular.z = self.turn_speed
            else:
                command.angular.z = -self.turn_speed


            # Do not leave TURNING immediately after one clear scan.
            if (
                front >= self.clear_distance
                and
                front_left >= self.obstacle_distance
                and
                front_right >= self.obstacle_distance
            ):
                self.clear_scan_count += 1
            else:
                self.clear_scan_count = 0


            if self.clear_scan_count >= self.clear_scan_required:

                self.state = 'PASSING'
                self.clear_scan_count = 0

                command.linear.x = self.passing_forward_speed
                command.angular.z = 0.0

                self.get_logger().info(
                    'Path consistently clear. Entering PASSING.'
                )


        # =====================================================
        # PASSING
        # =====================================================

        elif self.state == 'PASSING':

            command.linear.x = self.passing_forward_speed
            command.angular.z = 0.0


            if self.turn_direction == 'LEFT':
                obstacle_side = right
            else:
                obstacle_side = left


            # If obstacle appears in front again, return to TURNING.
            if front < self.obstacle_distance:

                self.state = 'TURNING'
                self.clear_scan_count = 0

                command.linear.x = self.turning_forward_speed

                if self.turn_direction == 'LEFT':
                    command.angular.z = self.turn_speed
                else:
                    command.angular.z = -self.turn_speed


            else:

                if obstacle_side >= self.side_clear_distance:
                    self.clear_scan_count += 1
                else:
                    self.clear_scan_count = 0


                if self.clear_scan_count >= self.clear_scan_required:

                    self.state = 'FORWARD'
                    self.turn_direction = None
                    self.clear_scan_count = 0

                    command.linear.x = self.forward_speed
                    command.angular.z = 0.0

                    self.get_logger().info(
                        'Obstacle passed. Returning to FORWARD.'
                    )


        else:

            command.linear.x = 0.0
            command.angular.z = 0.0

            self.state = 'FORWARD'
            self.turn_direction = None
            self.clear_scan_count = 0


        self.get_logger().info(
            f'State={self.state} | '
            f'Front={front:.2f} | '
            f'FL={front_left:.2f} | '
            f'FR={front_right:.2f} | '
            f'Left={left:.2f} | '
            f'Right={right:.2f}'
        )

        self.cmd_vel_publisher.publish(command)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoider()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()