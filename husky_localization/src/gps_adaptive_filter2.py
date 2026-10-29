#!/usr/bin/env python3

import math
import copy
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class GPSAdaptiveFilter(Node):

    def __init__(self):
        super().__init__('gps_adaptive_filter')

        # ==========================================================
        # Topics
        # ==========================================================

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

        # ==========================================================
        # Motion detection
        # ==========================================================

        self.declare_parameter(
            'stationary_speed_threshold',
            0.03
        )

        self.declare_parameter(
            'moving_speed_threshold',
            0.05
        )

        self.declare_parameter(
            'stationary_time',
            0.5
        )

        # ==========================================================
        # GPS covariance
        # ==========================================================

        # Covariance used during normal motion.
        self.declare_parameter(
            'normal_xy_covariance',
            4.0
        )

        # Very high covariance when GPS is first observed while
        # stationary and has not settled.
        self.declare_parameter(
            'initial_stationary_covariance',
            1000.0
        )

        # Lowest covariance GPS is allowed to reach while
        # stationary after it has demonstrated stability.
        self.declare_parameter(
            'settled_stationary_covariance',
            25.0
        )

        # ==========================================================
        # GPS settling
        # ==========================================================

        # Number of stationary GPS samples retained for robust
        # estimation.
        self.declare_parameter(
            'stationary_window_size',
            30
        )

        # Minimum stationary samples before GPS is considered
        # "settled enough" to receive significant weight.
        self.declare_parameter(
            'minimum_settling_samples',
            10
        )

        # GPS position standard deviation below which we consider
        # the stationary GPS solution reasonably stable.
        self.declare_parameter(
            'settled_stddev',
            0.8
        )

        # How quickly the internal GPS estimate follows the
        # stationary median.
        #
        # Smaller = slower / smoother.
        self.declare_parameter(
            'stationary_alpha',
            0.10
        )

        # ==========================================================
        # Outlier handling
        # ==========================================================

        # A GPS measurement farther than this from the current
        # robust stationary estimate is treated as an outlier.
        self.declare_parameter(
            'stationary_outlier_threshold',
            5.0
        )

        # Maximum GPS jump during motion before we strongly
        # downweight the measurement.
        self.declare_parameter(
            'moving_jump_threshold',
            5.0
        )

        # ==========================================================
        # State
        # ==========================================================

        self.local_vx = 0.0
        self.local_vy = 0.0

        self.stationary_since = None
        self.is_stationary = False

        self.last_gps_x = None
        self.last_gps_y = None

        self.last_gps_time = None

        # Smoothed/learned GPS position.
        self.filtered_gps_x = None
        self.filtered_gps_y = None

        # Recent stationary GPS measurements.
        self.stationary_x = deque(
            maxlen=self.get_parameter(
                'stationary_window_size'
            ).value
        )

        self.stationary_y = deque(
            maxlen=self.get_parameter(
                'stationary_window_size'
            ).value
        )

        # Number of samples collected since entering stationary
        # mode.
        self.stationary_samples = 0

        # ==========================================================
        # Parameters
        # ==========================================================

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

        self.initial_stationary_covariance = self.get_parameter(
            'initial_stationary_covariance'
        ).value

        self.settled_stationary_covariance = self.get_parameter(
            'settled_stationary_covariance'
        ).value

        self.minimum_settling_samples = self.get_parameter(
            'minimum_settling_samples'
        ).value

        self.settled_stddev = self.get_parameter(
            'settled_stddev'
        ).value

        self.stationary_alpha = self.get_parameter(
            'stationary_alpha'
        ).value

        self.stationary_outlier_threshold = self.get_parameter(
            'stationary_outlier_threshold'
        ).value

        self.moving_jump_threshold = self.get_parameter(
            'moving_jump_threshold'
        ).value

        # ==========================================================
        # ROS
        # ==========================================================

        gps_topic = self.get_parameter(
            'gps_topic'
        ).value

        local_odom_topic = self.get_parameter(
            'local_odom_topic'
        ).value

        output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.gps_sub = self.create_subscription(
            Odometry,
            gps_topic,
            self.gps_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            local_odom_topic,
            self.odom_callback,
            10
        )

        self.gps_pub = self.create_publisher(
            Odometry,
            output_topic,
            10
        )

        self.get_logger().info(
            'Adaptive GPS filter started'
        )

    # ==============================================================
    # LOCAL ODOMETRY
    # ==============================================================

    def odom_callback(self, msg):

        self.local_vx = msg.twist.twist.linear.x
        self.local_vy = msg.twist.twist.linear.y

        speed = math.hypot(
            self.local_vx,
            self.local_vy
        )

        now = self.get_clock().now()

        # ----------------------------------------------------------
        # Robot stopped
        # ----------------------------------------------------------

        if speed < self.stationary_speed_threshold:

            if self.stationary_since is None:
                self.stationary_since = now

            elapsed = (
                now - self.stationary_since
            ).nanoseconds / 1e9

            if elapsed >= self.stationary_time:

                if not self.is_stationary:

                    self.get_logger().info(
                        'Robot entered stationary mode'
                    )

                    self.enter_stationary_mode()

                self.is_stationary = True

        # ----------------------------------------------------------
        # Robot moving
        # ----------------------------------------------------------

        elif speed > self.moving_speed_threshold:

            if self.is_stationary:

                self.get_logger().info(
                    'Robot left stationary mode'
                )

            self.is_stationary = False
            self.stationary_since = None

            self.stationary_samples = 0

            self.stationary_x.clear()
            self.stationary_y.clear()

    # ==============================================================
    # STATIONARY MODE
    # ==============================================================

    def enter_stationary_mode(self):

        self.stationary_samples = 0

        self.stationary_x.clear()
        self.stationary_y.clear()

        # Do NOT reset filtered_gps_x/y.
        #
        # This is intentional.
        #
        # If the robot stops again later, we don't want to
        # introduce a discontinuity in the GPS estimate.

    # ==============================================================
    # GPS CALLBACK
    # ==============================================================

    def gps_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        now = self.get_clock().now()

        # ----------------------------------------------------------
        # First GPS measurement
        # ----------------------------------------------------------

        if self.filtered_gps_x is None:

            self.filtered_gps_x = x
            self.filtered_gps_y = y

            self.last_gps_x = x
            self.last_gps_y = y
            self.last_gps_time = now

            self.publish_gps(
                msg,
                x,
                y,
                self.initial_stationary_covariance
                if self.is_stationary
                else self.normal_xy_covariance
            )

            return

        # ----------------------------------------------------------
        # GPS displacement
        # ----------------------------------------------------------

        gps_dx = x - self.last_gps_x
        gps_dy = y - self.last_gps_y

        gps_jump = math.hypot(
            gps_dx,
            gps_dy
        )

        dt = (
            now - self.last_gps_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 0.001

        # ==========================================================
        # STATIONARY
        # ==========================================================

        if self.is_stationary:

            self.process_stationary_gps(
                msg,
                x,
                y
            )

        # ==========================================================
        # MOVING
        # ==========================================================

        else:

            self.process_moving_gps(
                msg,
                x,
                y,
                gps_jump
            )

        self.last_gps_x = x
        self.last_gps_y = y
        self.last_gps_time = now

    # ==============================================================
    # STATIONARY GPS
    # ==============================================================

    def process_stationary_gps(
        self,
        msg,
        x,
        y
    ):

        # ----------------------------------------------------------
        # If GPS estimate already exists, reject only gross
        # outliers from the stationary estimator.
        # ----------------------------------------------------------

        if self.filtered_gps_x is not None:

            distance_from_estimate = math.hypot(
                x - self.filtered_gps_x,
                y - self.filtered_gps_y
            )

            if (
                distance_from_estimate >
                self.stationary_outlier_threshold
            ):

                self.get_logger().warn(
                    'Stationary GPS outlier rejected: '
                    f'{distance_from_estimate:.2f} m from estimate'
                )

                return

        # ----------------------------------------------------------
        # Store measurement
        # ----------------------------------------------------------

        self.stationary_x.append(x)
        self.stationary_y.append(y)

        self.stationary_samples += 1

        # ----------------------------------------------------------
        # Robust stationary estimate
        # ----------------------------------------------------------

        median_x = self.median(
            self.stationary_x
        )

        median_y = self.median(
            self.stationary_y
        )

        # ----------------------------------------------------------
        # Slowly move our GPS estimate toward the robust median.
        #
        # This is the critical part.
        #
        # GPS:
        #
        # 0 -> 1 -> 2 -> 2.5 -> 2.4 -> 2.43
        #
        # filtered GPS moves gradually rather than jumping.
        # ----------------------------------------------------------

        alpha = self.stationary_alpha

        self.filtered_gps_x = (
            (1.0 - alpha) * self.filtered_gps_x
            + alpha * median_x
        )

        self.filtered_gps_y = (
            (1.0 - alpha) * self.filtered_gps_y
            + alpha * median_y
        )

        # ----------------------------------------------------------
        # Calculate GPS stability
        # ----------------------------------------------------------

        std_x = self.standard_deviation(
            self.stationary_x
        )

        std_y = self.standard_deviation(
            self.stationary_y
        )

        gps_std = max(
            std_x,
            std_y
        )

        # ----------------------------------------------------------
        # Adaptive covariance
        # ----------------------------------------------------------

        covariance = self.stationary_covariance(
            gps_std
        )

        self.publish_gps(
            msg,
            self.filtered_gps_x,
            self.filtered_gps_y,
            covariance
        )

        # ----------------------------------------------------------
        # Diagnostics
        # ----------------------------------------------------------

        if (
            self.stationary_samples %
            10 == 0
        ):

            self.get_logger().info(
                'Stationary GPS: '
                f'samples={self.stationary_samples}, '
                f'median=({median_x:.2f}, {median_y:.2f}), '
                f'std={gps_std:.2f} m, '
                f'covariance={covariance:.2f}'
            )

    # ==============================================================
    # MOVING GPS
    # ==============================================================

    def process_moving_gps(
        self,
        msg,
        x,
        y,
        gps_jump
    ):

        # ----------------------------------------------------------
        # During motion, use GPS normally but keep a modest
        # temporal smoothing so a single GPS spike doesn't create
        # a large jump.
        # ----------------------------------------------------------

        if gps_jump > self.moving_jump_threshold:

            self.get_logger().warn(
                'Large GPS jump detected: '
                f'{gps_jump:.2f} m'
            )

            # Instead of completely rejecting it, give it very
            # high covariance.
            #
            # This allows the EKF's own prediction and Mahalanobis
            # gating to dominate.
            covariance = max(
                self.normal_xy_covariance * 100.0,
                1000.0
            )

            self.publish_gps(
                msg,
                x,
                y,
                covariance
            )

            return

        # ----------------------------------------------------------
        # Normal moving GPS
        # ----------------------------------------------------------

        alpha = 0.35

        self.filtered_gps_x = (
            (1.0 - alpha) * self.filtered_gps_x
            + alpha * x
        )

        self.filtered_gps_y = (
            (1.0 - alpha) * self.filtered_gps_y
            + alpha * y
        )

        self.publish_gps(
            msg,
            self.filtered_gps_x,
            self.filtered_gps_y,
            self.normal_xy_covariance
        )

    # ==============================================================
    # ADAPTIVE COVARIANCE
    # ==============================================================

    def stationary_covariance(
        self,
        gps_std
    ):

        # ----------------------------------------------------------
        # Not enough data yet.
        # ----------------------------------------------------------

        if (
            self.stationary_samples <
            self.minimum_settling_samples
        ):

            return self.initial_stationary_covariance

        # ----------------------------------------------------------
        # GPS still unstable.
        #
        # Map GPS stddev onto covariance.
        # ----------------------------------------------------------

        if gps_std > self.settled_stddev:

            covariance = (
                gps_std * gps_std * 10.0
            )

            return max(
                covariance,
                self.settled_stationary_covariance
            )

        # ----------------------------------------------------------
        # GPS has settled.
        #
        # Slowly approach our minimum stationary covariance.
        # ----------------------------------------------------------

        return self.settled_stationary_covariance

    # ==============================================================
    # STATISTICS
    # ==============================================================

    @staticmethod
    def median(values):

        if not values:
            return 0.0

        data = sorted(values)

        n = len(data)

        middle = n // 2

        if n % 2 == 0:

            return (
                data[middle - 1]
                + data[middle]
            ) / 2.0

        return data[middle]

    @staticmethod
    def standard_deviation(values):

        if len(values) < 2:
            return 999.0

        mean = sum(values) / len(values)

        variance = sum(
            (v - mean) ** 2
            for v in values
        ) / (len(values) - 1)

        return math.sqrt(variance)

    # ==============================================================
    # PUBLISH
    # ==============================================================

    def publish_gps(
        self,
        msg,
        x,
        y,
        covariance
    ):

        output = copy.deepcopy(msg)

        output.pose.pose.position.x = x
        output.pose.pose.position.y = y

        # X variance
        output.pose.covariance[0] = covariance

        # Y variance
        output.pose.covariance[7] = covariance

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
