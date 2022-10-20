#! /usr/bin/env python3

# General Dependancies
import rospy
from std_msgs.msg import Int32  # Message definition


class publisherClass():
    def __init__(self):
        topic_name = 'counter'

        self.msg = Int32() # Create a "blank" message

        self.increment = 0 # Variable for information

        self.rate = rospy.Rate(1.0) # Hz, how often to publish data

        self.int_publisher = rospy.Publisher(topic_name, Int32, queue_size=1)

        while not rospy.is_shutdown():
            
            # Update message
            # FILL IN

            # Publish message
            # FILL IN

            # Increment value by 1
            # FILL IN

            # Wait until next publish cycle time
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("example_publisher_node")
        rospy.loginfo("Starting Example Publisher Node")
        s = publisherClass()
        rospy.spin()
    except rospy.ROSInterruptException: pass
