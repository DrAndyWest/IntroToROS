#! /usr/bin/env python3

# General Dependancies
import rospy
from std_srvs.srv import Trigger, TriggerResponse  # Service definition


class serviceClass():
    def __init__(self):
        node_name = rospy.get_name()
        s = rospy.Service(node_name + "/trigger", Trigger, self.service_handler) # Service, with callback function

        self.successMessage = "Trigger Complete"



    def service_handler(self, request):
        rospy.loginfo("Service request received")
        # No information is passed with a Trigger request, nothing to process/hand off to other function

        # Complete the response information
        response = TriggerResponse()
        response.success = True
        response.message = self.successMessage # Using "self" allows functions to access variables outside of themselves

        return response


if __name__ == "__main__":
    try:
        rospy.init_node("example_service_node")
        rospy.loginfo("Starting Example Service Node")
        s = serviceClass()
        rospy.spin()
    except rospy.ROSInterruptException: pass
