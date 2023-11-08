#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String, Time
from ros_upgrade_example.msg import ExampleMsg

node_name: str = "talker_node"
version: str = "0.1"

class TalkerNode:
    def __init__(self) -> None:
        rospy.init_node(node_name)
        rospy.loginfo(f"[{rospy.get_caller_id()}] Starting {node_name} v{version}...")

        self.publisher_chatter: rospy.Publisher = rospy.Publisher("~chatter", String, queue_size=10)

        publisher_latched: rospy.Publisher = rospy.Publisher("~startup_time", Time, queue_size=1, latch=True)
        publisher_latched.publish(rospy.get_rostime())

        self.should_publish: bool = True
        rospy.Subscriber("~should_publish", Bool, self.callback_should_publish)

        rospy.Subscriber("~new_msg", ExampleMsg, self.callback_new_msg)
        
        self.rate: rospy.Rate = rospy.Rate(10)

    def spin(self) -> None:
        rospy.loginfo(f"[{rospy.get_caller_id()}] Spinning...")
        while not rospy.is_shutdown():
            if self.should_publish:
                message_prefix: str = rospy.get_param("~message_prefix", "The time is")
                self.publisher_chatter.publish(f"{message_prefix}: {rospy.get_time()}")
            
            self.rate.sleep()

    def callback_should_publish(self, message: Bool) -> None:
        self.should_publish = message.data
    
    def callback_new_msg(self, message: ExampleMsg) -> None:
        rospy.loginfo(f"[{rospy.get_caller_id()}] Received a very important number: {message.important_number}")


if __name__ == "__main__":
    node: TalkerNode = TalkerNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
