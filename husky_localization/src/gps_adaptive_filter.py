#!/usr/bin/env python3

import math
import copy

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class GPSAdaptiveFilter(Node):

    def __init__(self):
        super().__init__('gps_adaptive_filter')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'gps_topic',
            '/odometry/gps'
        )

        self.declare_parameter(
            'local_odom_topic',
            '/a300_00063/platform/odom/filtered_sim_time'
        )

        self.declare_parameter(
            'output_topic',
            '/odometry/gps/filtered'
        )

        # Speed below this is considered stationary.
        self.declare_parameter(
            'stationary_speed_threshold',
            0.03
        )

        # Speed above this is considered moving.
        self.declare_parameter(
            'moving_speed_threshold',
            0.05
        )

        # How long speed must remain below stationary threshold.
        self.declare_parameter(
            'stationary_time',
            0.5
        )

        # Normal GPS X/Y covariance.
        self.declare_parameter(
            'normal_xy_covariance',
            4.0
        )

        # GPS X/Y covariance while stationary.
        #
        # 1000 m² means the EKF considers the GPS position
        # extremely uncertain while the robot is stopped.
        self.declare_parameter(
            'stationary_xy_covariance',
            1000.0
        )

        # Maximum GPS movement allowed while stationary.
        #
        # If GPS moves more than this between consecutive
        # measurements while the robot is stopped, reject it.
        self.declare_parameter(
            'stationary_gps_jump_threshold',
            0.5
        )

        # Maximum allowed GPS displacement beyond what local
        # odometry predicts.
        #
        # Example:
        #
        # local odom says robot moved 0.2 m
        # GPS moved 5 m
        #
        # => reject GPS.
        self.declare_parameter(
            'gps_jump_margin',
            2.0
        )

        # Absolute maximum GPS jump we will accept.
        self.declare_parameter(
            'maximum_gps_jump',
            10.0
        )

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.gps_topic = self.get_parameter(
            'gps_topic'
        ).value

        self.local_odom_topic = self.get_parameter(
            'local_odom_topic'
        ).value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.stationary_speed_threshold = self.get_parameter(
            'stationary_speed_threshold'
        ).value

        self.moving_speed_threshold = self.get_parameter(
            'moving_speed_threshold'
        ).value

        self.stationary_time = self.get_parameter(
            'stationary_time'
        ).value

        self.normal_xy_covariance = self.get_parameter(
            'normal_xy_covariance'
        ).value

        self.stationary_xy_covariance = self.get_parameter(
            'stationary_xy_covariance'
        ).value

        self.stationary_gps_jump_threshold = self.get_parameter(
            'stationary_gps_jump_threshold'
        ).value

        self.gps_jump_margin = self.get_parameter(
            'gps_jump_margin'
        ).value

        self.maximum_gps_jump = self.get_parameter(
            'maximum_gps_jump'
        ).value

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.last_gps_x = None
        self.last_gps_y = None

        self.last_gps_time = None

        self.local_vx = 0.0
        self.local_vy = 0.0

        self.stationary_since = None

        self.is_stationary = False

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.gps_sub = self.create_subscription(
            Odometry,
            self.gps_topic,
            self.gps_callback,
            10
        )

        self.local_odom_sub = self.create_subscription(
            Odometry,
            self.local_odom_topic,
            self.local_odom_callback,
            10
        )

        self.gps_pub = self.create_publisher(
            Odometry,
            self.output_topic,
            10
        )

        self.get_logger().info(
            'GPS adaptive filter started'
        )

        self.get_logger().info(
            f'GPS input: {self.gps_topic}'
        )

        self.get_logger().info(
            f'Local odom: {self.local_odom_topic}'
        )

        self.get_logger().info(
            f'GPS output: {self.output_topic}'
        )

    # =============================================================
    # Local odometry
    # =============================================================

    def local_odom_callback(self, msg):

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.local_vx = vx
        self.local_vy = vy

        speed = math.sqrt(
            vx * vx +
            vy * vy
        )

        now = self.get_clock().now()

        # ---------------------------------------------------------
        # Stationary detection
        # ---------------------------------------------------------

        if speed < self.stationary_speed_threshold:

            if self.stationary_since is None:
                self.stationary_since = now

            stationary_duration = (
                now - self.stationary_since
            ).nanoseconds / 1e9

            if stationary_duration >= self.stationary_time:

                if not self.is_stationary:
                    self.get_logger().info(
                        f'Robot stationary '
                        f'(speed={speed:.3f} m/s)'
                    )

                self.is_stationary = True

        elif speed > self.moving_speed_threshold:

            if self.is_stationary:
                self.get_logger().info(
                    f'Robot moving '
                    f'(speed={speed:.3f} m/s)'
                )

            self.is_stationary = False
            self.stationary_since = None

    # =============================================================
    # GPS
    # =============================================================

    def gps_callback(self, msg):

        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y

        current_time = self.get_clock().now()

        # ---------------------------------------------------------
        # First GPS message
        # ---------------------------------------------------------

        if self.last_gps_x is None:

            self.last_gps_x = gps_x
            self.last_gps_y = gps_y

            self.last_gps_time = current_time

            self.publish_gps(
                msg,
                self.normal_xy_covariance
            )

            self.get_logger().info(
                f'Initial GPS position: '
                f'x={gps_x:.3f}, y={gps_y:.3f}'
            )

            return

        # ---------------------------------------------------------
        # GPS displacement
        # ---------------------------------------------------------

        gps_dx = gps_x - self.last_gps_x
        gps_dy = gps_y - self.last_gps_y

        gps_displacement = math.sqrt(
            gps_dx * gps_dx +
            gps_dy * gps_dy
        )

        dt = (
            current_time - self.last_gps_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 0.001

        # ---------------------------------------------------------
        # Local odometry predicted displacement
        # ---------------------------------------------------------

        local_speed = math.sqrt(
            self.local_vx * self.local_vx +
            self.local_vy * self.local_vy
        )

        expected_local_displacement = (
            local_speed * dt
        )

        # ---------------------------------------------------------
        # GPS jump rejection
        # ---------------------------------------------------------

        reject = False
        rejection_reason = ''

        # Case 1:
        # Robot is stationary and GPS moved significantly.
        #
        # Example:
        #
        # robot speed = 0
        # GPS: 0 -> 1 -> 2 -> 3 m
        #
        # These measurements are rejected.
        if (
            self.is_stationary
            and
            gps_displacement >
            self.stationary_gps_jump_threshold
        ):

            reject = True

            rejection_reason = (
                f'robot stationary, '
                f'GPS moved {gps_displacement:.2f} m'
            )

        # Case 2:
        # GPS moved much farther than local odometry says
        # the robot could have moved.
        elif (
            gps_displacement >
            expected_local_displacement +
            self.gps_jump_margin
        ):

            reject = True

            rejection_reason = (
                f'GPS moved {gps_displacement:.2f} m, '
                f'local odom predicts '
                f'{expected_local_displacement:.2f} m'
            )

        # Case 3:
        # Absolute sanity limit.
        elif gps_displacement > self.maximum_gps_jump:

            reject = True

            rejection_reason = (
                f'GPS jump {gps_displacement:.2f} m '
                f'exceeds maximum allowed'
            )

        # ---------------------------------------------------------
        # Reject GPS
        # ---------------------------------------------------------

        if reject:

            self.get_logger().warn(
                f'Rejecting GPS measurement: '
                f'{rejection_reason}'
            )

            # IMPORTANT:
            #
            # We DO NOT update last_gps_x/y when rejecting.
            #
            # This means the next GPS measurement is still
            # compared against the last accepted GPS position.
            #
            # This prevents a bad GPS excursion from becoming
            # the new reference point.

            return

        # ---------------------------------------------------------
        # Adaptive covariance
        # ---------------------------------------------------------

        if self.is_stationary:

            covariance = (
                self.stationary_xy_covariance
            )

        else:

            covariance = (
                self.normal_xy_covariance
            )

        # ---------------------------------------------------------
        # Publish accepted GPS
        # ---------------------------------------------------------

        self.publish_gps(
            msg,
            covariance
        )

        # Update accepted GPS reference
        self.last_gps_x = gps_x
        self.last_gps_y = gps_y
        self.last_gps_time = current_time

    # =============================================================
    # Publish
    # =============================================================

    def publish_gps(
        self,
        msg,
        xy_covariance
    ):

        output = copy.deepcopy(msg)

        # X variance
        output.pose.covariance[0] = (
            xy_covariance
        )

        # Y variance
        output.pose.covariance[7] = (
            xy_covariance
        )

        self.gps_pub.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = GPSAdaptiveFilter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
