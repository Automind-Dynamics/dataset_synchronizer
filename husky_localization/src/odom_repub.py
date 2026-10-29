#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class OdomTimestampOverride(Node):

    def __init__(self):
        super().__init__('odom_timestamp_override')

        self.latest_clock = None

        # /clock uses Best Effort QoS
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/a300_00063/platform/odom/filtered',
            self.odom_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/a300_00063/platform/odom/filtered_sim_time',
            10
        )

        self.get_logger().info(
            'Odom timestamp override node started'
        )

    def clock_callback(self, msg: Clock):
        self.latest_clock = msg.clock

    def odom_callback(self, msg: Odometry):

        if self.latest_clock is None:
            self.get_logger().warn(
                'Waiting for /clock...',
                throttle_duration_sec=5.0
            )
            return

        odom = copy.deepcopy(msg)

        # Replace odometry timestamp with latest simulation clock
        odom.header.stamp = self.latest_clock

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = OdomTimestampOverride()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
