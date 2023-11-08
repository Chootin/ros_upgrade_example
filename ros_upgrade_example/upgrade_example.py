#!/usr/bin/env python

import rclpy
from rclpy.node import Node, Publisher, Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time
from ros_upgrade_example.msg import ExampleMsg

node_name: str = "talker_node"
version: str = "0.1"

class TalkerNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name)
        self.get_logger().info(f"Starting {node_name} v{version}...")

        self.declare_parameter("message_prefix", "The time is")

        self.publisher_chatter: Publisher = self.create_publisher(String, "~/chatter", qos_profile=10)

        qos_profile_latched: QoSProfile = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, depth=1)
        publisher_latched: Publisher = self.create_publisher(Time, "~/startup_time", qos_profile=qos_profile_latched)
        publisher_latched.publish(self.get_clock().now().to_msg())

        self.should_publish: bool = True
        self.create_subscription(Bool, "~/should_publish", self.callback_should_publish, 1)

        self.create_subscription(ExampleMsg, "~/new_msg", self.callback_new_msg, 1)

        self.create_timer(0.1, self.on_timer)

    def on_timer(self) -> None:
        if self.should_publish:
            parameter_prefix: Parameter = self.get_parameter("message_prefix")
            self.publisher_chatter.publish(String(data=f"{parameter_prefix.get_parameter_value().string_value}: {self.get_clock().now()}"))

    def callback_should_publish(self, message: Bool) -> None:
        self.should_publish = message.data
    
    def callback_new_msg(self, message: ExampleMsg) -> None:
        self.get_logger().info(f"Received a very important number: {message.important_number}")


if __name__ == "__main__":
    rclpy.init()
    node: TalkerNode = TalkerNode()
    node.get_logger().info("Spinning...")
    rclpy.spin(node)
