#! /usr/bin/env python3

# General Dependancies
import rospy
from std_msgs.msg import Int32  # Message definition


class subscriberClass():
    def __init__(self):
        topic_name = 'counter'

        rospy.Subscriber(topic_name, Int32, self.subcallback)

    # FILL IN CALLBACK FUNCTION



if __name__ == "__main__":
    try:
        rospy.init_node("example_subscriber_node")
        rospy.loginfo("Starting Example Subscriber Node")
        s = subscriberClass()
        rospy.spin()
    except rospy.ROSInterruptException: pass
