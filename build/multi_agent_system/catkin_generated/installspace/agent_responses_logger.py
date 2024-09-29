#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from termcolor import colored
import signal
import sys
import os
from datetime import datetime

class AgentResponsesLogger:
    def __init__(self):
        rospy.init_node('agent_responses_logger', anonymous=True)
        
        # Create the responses directory if it doesn't exist
        self.responses_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'responses')
        os.makedirs(self.responses_dir, exist_ok=True)
        
        # Create a new log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(self.responses_dir, f"agent_responses_{timestamp}.log")
        self.log_file = open(self.log_file_path, 'w')
        
        # Subscribe to topics for each agent
        rospy.Subscriber('/user_feedback', String, self.log_manager_response)
        rospy.Subscriber('/structural_engineer_feedback', String, self.log_structural_engineer_response)
        rospy.Subscriber('/stability_feedback', String, self.log_stability_response)
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        rospy.loginfo(f"Agent Responses Logger initialized. Logging to {self.log_file_path}")

    def log_manager_response(self, msg):
        self.log_response("Manager Agent", msg.data, 'blue')

    def log_structural_engineer_response(self, msg):
        self.log_response("Structural Engineer Agent", msg.data, 'green')

    def log_stability_response(self, msg):
        self.log_response("Stability Agent", msg.data, 'yellow')

    def log_response(self, agent_name, message, color):
        formatted_message = f"\n{'-'*80}\n{agent_name}: {message}\n{'-'*80}\n"
        print(colored(formatted_message, color))
        rospy.loginfo(formatted_message)
        self.log_file.write(formatted_message)
        self.log_file.flush()

    def signal_handler(self, signum, frame):
        rospy.loginfo("Shutdown signal received. Cleaning up...")
        self.log_file.close()
        rospy.loginfo(f"Responses logged to {self.log_file_path}")
        rospy.signal_shutdown("Received shutdown signal")
        sys.exit(0)

if __name__ == '__main__':
    try:
        logger = AgentResponsesLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
