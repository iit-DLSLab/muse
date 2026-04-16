#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("point_lio_odom_bridge")

        self.declare_parameter("input_topic", "/point_lio/odometry")
        self.declare_parameter("output_topic", "/lidar_odometry")
        self.declare_parameter("use_input_stamp", True)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "")

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.use_input_stamp = self.get_parameter("use_input_stamp").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value

        self.pub = self.create_publisher(Odometry, self.output_topic, 50)
        self.sub = self.create_subscription(Odometry, self.input_topic, self.callback, 50)

        self.get_logger().info(
            f"Point-LIO odom bridge: {self.input_topic} -> {self.output_topic}"
        )

    def callback(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if not self.use_input_stamp:
            out.header.stamp = self.get_clock().now().to_msg()

        if self.frame_id:
            out.header.frame_id = self.frame_id

        if self.child_frame_id:
            out.child_frame_id = self.child_frame_id

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
