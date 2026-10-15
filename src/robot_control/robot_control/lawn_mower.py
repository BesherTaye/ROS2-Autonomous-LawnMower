import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy
)

from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener


class LawnMowerNavigator(Node):

    def __init__(self):
        super().__init__('lawn_mower_navigator')

        # =====================================================
        # NAV2 ACTION CLIENT
        # =====================================================

        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # =====================================================
        # COVERAGE AREA
        # =====================================================
        #
        # Approximate usable area of the map.
        #
        # We intentionally stay away from the extreme wall
        # boundaries so that:
        #
        #   - the robot footprint does not collide with walls
        #   - Nav2 inflation has enough room
        #   - localization error does not push the robot outside
        #
        # Change these values later if necessary.

        self.x_min = -3.20
        self.x_max = 3.20

        self.y_min = -3.20
        self.y_max = 3.20

        # Distance between adjacent coverage lanes.
        #
        # Smaller value:
        #   - denser coverage
        #   - longer mission
        #
        # Larger value:
        #   - faster mission
        #   - less overlap
        self.lane_spacing = 0.55

        # =====================================================
        # MISSION STATE
        # =====================================================
        #
        # phase:
        #
        #   0 = vertical coverage
        #   1 = horizontal coverage
        #   2 = mission finished

        self.phase = 0

        self.navigation_started = False

        # =====================================================
        # TF
        # =====================================================
        #
        # Used for recording the actual robot trajectory:
        #
        #     map -> base_link

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # TRANSIENT-LOCAL QOS FOR INTENDED PATHS
        # =====================================================
        #
        # This allows RViz to receive the path even if RViz
        # starts after the path was originally published.

        intended_path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =====================================================
        # VERTICAL INTENDED PATH PUBLISHER
        # =====================================================

        self.vertical_path_publisher = self.create_publisher(
            Path,
            '/vertical_coverage_path',
            intended_path_qos
        )

        # =====================================================
        # HORIZONTAL INTENDED PATH PUBLISHER
        # =====================================================

        self.horizontal_path_publisher = self.create_publisher(
            Path,
            '/horizontal_coverage_path',
            intended_path_qos
        )

        # =====================================================
        # ACTUAL ROBOT PATH PUBLISHER
        # =====================================================

        self.actual_path_publisher = self.create_publisher(
            Path,
            '/actual_path',
            10
        )

        self.actual_path = Path()

        self.actual_path.header.frame_id = 'map'

        # =====================================================
        # GENERATE BOTH COVERAGE PATTERNS
        # =====================================================

        self.vertical_poses = (
            self.generate_vertical_serpentine()
        )

        self.horizontal_poses = (
            self.generate_horizontal_serpentine()
        )

        self.get_logger().info(
            f'Generated {len(self.vertical_poses)} '
            'vertical coverage poses.'
        )

        self.get_logger().info(
            f'Generated {len(self.horizontal_poses)} '
            'horizontal coverage poses.'
        )

        # =====================================================
        # PUBLISH INTENDED PATHS
        # =====================================================

        self.publish_intended_path(
            self.vertical_poses,
            self.vertical_path_publisher
        )

        self.publish_intended_path(
            self.horizontal_poses,
            self.horizontal_path_publisher
        )

        # =====================================================
        # ACTUAL PATH TIMER
        # =====================================================
        #
        # Record robot position 5 times per second.

        self.path_timer = self.create_timer(
            0.2,
            self.record_actual_path
        )

        # =====================================================
        # START MISSION TIMER
        # =====================================================
        #
        # Give Nav2 / AMCL / TF a short time to settle.

        self.start_timer = self.create_timer(
            2.0,
            self.start_navigation
        )

        self.get_logger().info(
            'Crosshatch lawn-mower coverage node started.'
        )

        self.get_logger().info(
            'RViz topics:'
        )

        self.get_logger().info(
            '  /vertical_coverage_path'
        )

        self.get_logger().info(
            '  /horizontal_coverage_path'
        )

        self.get_logger().info(
            '  /actual_path'
        )


    # =========================================================
    # CREATE POSE
    # =========================================================

    def make_pose(self, x, y, yaw):

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert planar yaw angle to quaternion.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0

        pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        return pose


    # =========================================================
    # GENERATE VERTICAL SERPENTINE
    # =========================================================

    def generate_vertical_serpentine(self):

        poses = []

        x = self.x_min

        going_up = True

        while x <= self.x_max + 1e-6:

            if going_up:

                start_y = self.y_min
                end_y = self.y_max

                yaw = math.pi / 2.0

            else:

                start_y = self.y_max
                end_y = self.y_min

                yaw = -math.pi / 2.0

            # Start of vertical lane.
            poses.append(
                self.make_pose(
                    x,
                    start_y,
                    yaw
                )
            )

            # End of vertical lane.
            poses.append(
                self.make_pose(
                    x,
                    end_y,
                    yaw
                )
            )

            # Move sideways to next lane.
            x += self.lane_spacing

            # Reverse direction.
            going_up = not going_up

        return poses


    # =========================================================
    # GENERATE HORIZONTAL SERPENTINE
    # =========================================================

    def generate_horizontal_serpentine(self):

        poses = []

        y = self.y_min

        going_right = True

        while y <= self.y_max + 1e-6:

            if going_right:

                start_x = self.x_min
                end_x = self.x_max

                yaw = 0.0

            else:

                start_x = self.x_max
                end_x = self.x_min

                yaw = math.pi

            # Start of horizontal lane.
            poses.append(
                self.make_pose(
                    start_x,
                    y,
                    yaw
                )
            )

            # End of horizontal lane.
            poses.append(
                self.make_pose(
                    end_x,
                    y,
                    yaw
                )
            )

            # Move upward to next lane.
            y += self.lane_spacing

            # Reverse direction.
            going_right = not going_right

        return poses


    # =========================================================
    # PUBLISH INTENDED COVERAGE PATH
    # =========================================================

    def publish_intended_path(
        self,
        poses,
        publisher
    ):

        path = Path()

        path.header.frame_id = 'map'

        path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Copy poses into Path message.
        for original_pose in poses:

            pose = PoseStamped()

            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp

            pose.pose = original_pose.pose

            path.poses.append(pose)

        publisher.publish(path)


    # =========================================================
    # RECORD ACTUAL ROBOT PATH
    # =========================================================

    def record_actual_path(self):

        if not self.navigation_started:
            return

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

        except Exception:
            return

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Translation.
        pose.pose.position.x = (
            transform.transform.translation.x
        )

        pose.pose.position.y = (
            transform.transform.translation.y
        )

        pose.pose.position.z = (
            transform.transform.translation.z
        )

        # Orientation.
        pose.pose.orientation = (
            transform.transform.rotation
        )

        # Store actual robot pose.
        self.actual_path.poses.append(pose)

        self.actual_path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Publish complete historical path.
        self.actual_path_publisher.publish(
            self.actual_path
        )


    # =========================================================
    # START NAVIGATION
    # =========================================================

    def start_navigation(self):

        if self.navigation_started:
            return

        self.navigation_started = True

        self.start_timer.cancel()

        self.get_logger().info(
            'Waiting for Nav2 NavigateThroughPoses server...'
        )

        self.nav_client.wait_for_server()

        self.get_logger().info(
            'Nav2 available.'
        )

        self.start_vertical_phase()


    # =========================================================
    # START VERTICAL PHASE
    # =========================================================

    def start_vertical_phase(self):

        self.phase = 0

        self.get_logger().info(
            '==========================================='
        )

        self.get_logger().info(
            'STARTING VERTICAL SERPENTINE COVERAGE'
        )

        self.get_logger().info(
            f'{len(self.vertical_poses)} poses'
        )

        self.get_logger().info(
            '==========================================='
        )

        self.send_coverage_goal(
            self.vertical_poses
        )


    # =========================================================
    # START HORIZONTAL PHASE
    # =========================================================

    def start_horizontal_phase(self):

        self.phase = 1

        self.get_logger().info(
            '==========================================='
        )

        self.get_logger().info(
            'VERTICAL PASS FINISHED'
        )

        self.get_logger().info(
            'STARTING HORIZONTAL SERPENTINE COVERAGE'
        )

        self.get_logger().info(
            f'{len(self.horizontal_poses)} poses'
        )

        self.get_logger().info(
            '==========================================='
        )

        self.send_coverage_goal(
            self.horizontal_poses
        )


    # =========================================================
    # SEND NAVIGATE-THROUGH-POSES GOAL
    # =========================================================

    def send_coverage_goal(self, poses):

        goal = NavigateThroughPoses.Goal()

        stamp = (
            self.get_clock().now().to_msg()
        )

        goal_poses = []

        # Create fresh stamped poses.
        for original_pose in poses:

            pose = PoseStamped()

            pose.header.frame_id = 'map'
            pose.header.stamp = stamp

            pose.pose = original_pose.pose

            goal_poses.append(pose)

        goal.poses = goal_poses

        future = (
            self.nav_client.send_goal_async(goal)
        )

        future.add_done_callback(
            self.goal_response_callback
        )


    # =========================================================
    # NAV2 GOAL RESPONSE
    # =========================================================

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Coverage goal rejected by Nav2.'
            )

            return

        if self.phase == 0:

            self.get_logger().info(
                'Vertical coverage mission accepted.'
            )

        elif self.phase == 1:

            self.get_logger().info(
                'Horizontal coverage mission accepted.'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )


    # =========================================================
    # NAV2 RESULT
    # =========================================================

    def result_callback(self, future):

        wrapped_result = future.result()

        status = wrapped_result.status

        # =====================================================
        # VERTICAL COVERAGE FINISHED
        # =====================================================

        if self.phase == 0:

            if status == GoalStatus.STATUS_SUCCEEDED:

                self.get_logger().info(
                    'Vertical serpentine completed successfully.'
                )

            else:

                self.get_logger().warning(
                    f'Vertical serpentine finished '
                    f'with status={status}.'
                )

                self.get_logger().warning(
                    'Starting horizontal pass anyway.'
                )

            # Start second pass regardless.
            self.start_horizontal_phase()

            return

        # =====================================================
        # HORIZONTAL COVERAGE FINISHED
        # =====================================================

        if self.phase == 1:

            if status == GoalStatus.STATUS_SUCCEEDED:

                self.get_logger().info(
                    'Horizontal serpentine completed successfully.'
                )

            else:

                self.get_logger().warning(
                    f'Horizontal serpentine finished '
                    f'with status={status}.'
                )

            self.phase = 2

            self.get_logger().info(
                '==========================================='
            )

            self.get_logger().info(
                'CROSSHATCH COVERAGE COMPLETE'
            )

            self.get_logger().info(
                'VERTICAL + HORIZONTAL PASSES FINISHED'
            )

            self.get_logger().info(
                '==========================================='
            )


def main(args=None):

    rclpy.init(args=args)

    node = LawnMowerNavigator()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()