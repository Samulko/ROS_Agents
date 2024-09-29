#!/usr/bin/env python3

import rospy
from multi_agent_system.srv import PlanExecution, PlanExecutionRequest
from std_msgs.msg import String
import sys
import threading
import os
from dotenv import load_dotenv

class TestPlanningAgent:
    def __init__(self):
        load_dotenv()  # Load environment variables from .env file
        rospy.init_node('test_planning_agent', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("Test Planning Agent: Node initialized")
        
        self.test_completed = False
        self.test_result = None
        self.test_timer = None
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key:
            rospy.logerr("OPENAI_API_KEY not found in environment variables")
            sys.exit(1)

    def run_test(self):
        self.test_timer = threading.Timer(30.0, self.test_timeout)  # 30-second timeout
        self.test_timer.start()

        try:
            rospy.loginfo("Test Planning Agent: Waiting for /plan_execution service...")
            rospy.wait_for_service('/plan_execution', timeout=10)
            rospy.loginfo("Test Planning Agent: Service /plan_execution is available")
        except rospy.ROSException as e:
            self.test_result = f"Service /plan_execution not available within timeout: {e}"
            self.complete_test()
            return

        plan_execution_service = rospy.ServiceProxy('/plan_execution', PlanExecution)

        test_plan = """
        - Support the beam (element 2) to secure the structure.
        - Disconnect the vertical column (1) from below the beam (2), which is being supported.
        - The worker should carefully remove the disconnected column (1) using appropriate tools.
        - Carefully remove the disconnected column (3) using appropriate tools.
        - Finally, the robot removes the beam (2) that is being supported last.
        """

        try:
            rospy.loginfo("Test Planning Agent: Calling plan_execution service...")
            response = plan_execution_service(PlanExecutionRequest(plan=test_plan))
            self.test_result = f"Received response - Success: {response.success}, Details: {response.execution_details}"
        except rospy.ServiceException as e:
            self.test_result = f"Service call failed: {e}"
        except Exception as e:
            self.test_result = f"Unexpected error: {e}"

        self.complete_test()

    def complete_test(self):
        if not self.test_completed:
            self.test_completed = True
            if self.test_timer:
                self.test_timer.cancel()
            rospy.loginfo(f"Test Planning Agent: Test completed. Result: {self.test_result}")
            rospy.signal_shutdown("Test completed")

    def test_timeout(self):
        self.test_result = "Test timed out after 30 seconds"
        self.complete_test()

if __name__ == '__main__':
    try:
        test_agent = TestPlanningAgent()
        test_agent.run_test()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test Planning Agent: Interrupted")
    except Exception as e:
        rospy.logerr(f"Test Planning Agent: Unexpected error: {e}")
    finally:
        rospy.loginfo("Test Planning Agent: Shutting down")
        sys.exit(0)
