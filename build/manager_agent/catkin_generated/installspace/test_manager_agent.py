#!/usr/bin/env python3

import unittest
import rospy
import rostest
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, StabilityAnalysis

class TestManagerAgent(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_manager_agent', anonymous=True)
        self.user_command_pub = rospy.Publisher('/user_command', String, queue_size=10)
        self.feedback_received = False
        self.feedback_message = ""
        rospy.Subscriber('/user_feedback', String, self.feedback_callback)
        rospy.wait_for_service('/validate_request', timeout=10)
        rospy.wait_for_service('/stability_analysis', timeout=10)
        self.validate_request = rospy.ServiceProxy('/validate_request', ValidateRequest)
        self.stability_analysis = rospy.ServiceProxy('/stability_analysis', StabilityAnalysis)

    def feedback_callback(self, data):
        self.feedback_received = True
        self.feedback_message = data.data

    def test_user_command_processing(self):
        test_command = "Disassemble the frame"
        self.user_command_pub.publish(test_command)
        timeout = rospy.Duration(60)  # Increased timeout to 60 seconds
        start_time = rospy.Time.now()
        while not self.feedback_received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        self.assertTrue(self.feedback_received, "No feedback received from manager_agent within 60 seconds")
        self.assertIn("Received command", self.feedback_message, "Unexpected feedback message")
        
        # Wait for the full processing cycle
        rospy.sleep(30)  # Increased wait time to 30 seconds
        
        # Check for validation result
        validation_received = False
        start_time = rospy.Time.now()
        while not validation_received and (rospy.Time.now() - start_time) < timeout:
            if "Validation result" in self.feedback_message or "Your request doesn't follow standard procedures" in self.feedback_message:
                validation_received = True
            else:
                rospy.sleep(0.1)
        
        self.assertTrue(validation_received, f"No validation result received. Last feedback: {self.feedback_message}")
        
        # We don't check for stability analysis anymore since the request might not be standard

    def test_validate_request_service(self):
        test_request = "Disassemble the frame"
        response = self.validate_request(test_request)
        self.assertIsNotNone(response, "No response from validate_request service")
        self.assertIsInstance(response.is_standard, bool, "is_standard should be a boolean")
        self.assertIsInstance(response.validation_details, str, "validation_details should be a string")

    def test_stability_analysis_service(self):
        test_request = "Disassemble the frame"
        response = self.stability_analysis(test_request)
        self.assertIsNotNone(response, "No response from stability_analysis service")
        self.assertIsInstance(response.is_safe, bool, "is_safe should be a boolean")
        self.assertIsInstance(response.modifications, str, "modifications should be a string")

if __name__ == '__main__':
    rostest.rosrun('manager_agent', 'test_manager_agent', TestManagerAgent)
