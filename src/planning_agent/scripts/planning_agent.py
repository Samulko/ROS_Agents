#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from multi_agent_system.srv import PlanExecution, PlanExecutionResponse
from openai import OpenAI
from pydantic import Field
from typing import List, cast
import instructor
from instructor import OpenAISchema
from dotenv import load_dotenv
import os
import time

# Load environment variables from .env file
load_dotenv()

class RoboticAction(OpenAISchema):
    """
    Represents a single robotic action in the disassembly sequence.
    """
    human_working: bool = Field(..., description="Indicates if a human is working alongside the robot")
    selected_element: str = Field(..., description="The element being worked on")
    planning_sequence: List[str] = Field(..., description="List of actions for the robot to perform")

class ActionSequence(OpenAISchema):
    """
    Represents a sequence of robotic actions for the disassembly plan.
    """
    actions: List[RoboticAction] = Field(..., description="List of robotic actions to perform")

class PlanningAgent:
    def __init__(self):
        rospy.init_node('planning_agent', anonymous=False)

        # Parameters
        self.openai_api_key = rospy.get_param('/openai_api_key')

        # Initialize OpenAI client with Instructor
        openai_client = OpenAI(api_key=self.openai_api_key)
        self.client = cast(OpenAI, instructor.patch(openai_client))

        # Service to execute plans
        self.plan_execution_service = rospy.Service('/plan_execution', PlanExecution, self.handle_plan_execution)

        # Publisher to robot control interface
        self.robot_control_pub = rospy.Publisher('/robot_control_command', String, queue_size=10)

        # Dictionary of possible robot actions
        self.robot_actions = {
            "move_in_cartesian_path": "move_in_cartesian_path(move_distance_x, move_distance_y, move_distance_z)",
            "moveto": "moveto",
            "picking": "picking",
            "holding": "holding",
            "placing": "placing",
            "human_action": "human_action(action_description)"
        }

        rospy.loginfo("Planning Agent Initialized and ready to work.")

    def handle_plan_execution(self, req):
        plan = req.plan
        rospy.loginfo(f"Planning Agent: Received plan execution request: {plan}")

        # Translate plan into structured action sequences
        action_sequence = self.translate_plan(plan)

        if action_sequence is None:
            return PlanExecutionResponse(success=False, execution_details="Failed to generate a valid action sequence.")

        # Validate action sequence
        if self.validate_action_sequence(action_sequence):
            # Execute preliminary steps
            self.execute_preliminary_steps()

            # Check if additional safety measures are needed
            if "unsafe" in plan.lower() or "modifications" in plan.lower():
                action_sequence = self.add_safety_measures(action_sequence)

            # Write the JSON file
            json_file_path = self.write_json_file(action_sequence)

            # Execute actions
            success, execution_details = self.execute_actions(action_sequence)
            return PlanExecutionResponse(success=success, execution_details=f"{execution_details}. JSON file created at {json_file_path}")
        else:
            return PlanExecutionResponse(success=False, execution_details="Invalid action sequence. Please check the plan and try again.")

    def add_safety_measures(self, action_sequence):
        rospy.loginfo("Planning Agent: Adding additional safety measures to the action sequence")
        safety_measures = [
            "implement_temporary_supports",
            "distribute_load_evenly",
            "monitor_stability_continuously"
        ]
        for action in action_sequence.actions:
            action.planning_sequence = safety_measures + action.planning_sequence
        return action_sequence

    def translate_plan(self, plan):
        rospy.loginfo(f"Planning Agent: Translating plan: {plan}")
        action_schemas = ', '.join(f"{key}: {value}" for key, value in self.robot_actions.items())
        prompt = f"""
        You are the Planning Agent in a multi-agent system that controls a robotic arm for disassembly tasks. Your role is to translate the disassembly sequence plan into a structured action sequence for the robotic arm to execute, collaborating with a human operator. 

        Given the disassembly plan:
        {plan}

        Your task is to:
        1. Analyze the given disassembly plan, focusing primarily on the numbered Disassembly Instructions.
        2. Create a detailed action sequence using only the following action schemas:
           {action_schemas}
        3. Ensure the action sequence follows the EXACT order specified in the numbered Disassembly Instructions.
        4. Maintain consistent roles for each actor throughout the entire process as defined in the numbered instructions.
        5. Identify the specific element being worked on in each step.
        6. Ensure that if an actor is instructed to support an element, they continue to do so until explicitly instructed to release it.

        Guidelines:
        - Prioritize the numbered Disassembly Instructions over any additional comments or information provided.
        - Follow the disassembly instructions step by step, without changing the order or assigned roles.
        - Break down complex movements into a series of simpler actions.
        - Include necessary preparatory movements before each main action.
        - For human actions, use the format: human_action(action_description)
        - Use specific element names (e.g., "element_1" instead of "element 1") for consistency.
        - Use EXACTLY the action names provided (e.g., 'moveto' not 'move_to').
        - Set human_working to true for steps performed by humans, and false for steps performed by the robot.
        - When human_working is true, only include human_action in the planning_sequence.
        - When human_working is false, only include robot actions in the planning_sequence.
        - Ensure that each actor maintains their assigned role throughout the entire process as specified in the numbered instructions.

        Ensure that:
        1. "human_working" is set appropriately based on whether the action is performed by a human or the robot, as specified in the numbered instructions.
        2. "selected_element" specifies the element being worked on in the current step.
        3. The actions in the "planning_sequence" are organized in execution order.
        4. Robot pick-and-place sequences follow this pattern: moveto -> picking -> holding -> placing
        5. Support actions follow this pattern: moveto -> holding, and continue holding in subsequent steps
        6. Use 'deposition_zone' as the destination for removed elements.
        7. Each actor maintains their assigned role consistently throughout the entire sequence as per the numbered instructions.
        """
        try:
            action_sequence = self.client.chat.completions.create(
                model="gpt-4",
                response_model=ActionSequence,
                messages=[
                    {"role": "system", "content": "You are a planning agent that translates disassembly plans into structured action sequences."},
                    {"role": "user", "content": prompt}
                ]
            )
            rospy.loginfo(f"Planning Agent: Translated plan into action sequence: {action_sequence}")
            return action_sequence
        except Exception as e:
            rospy.logerr(f"Planning Agent: Failed to generate action sequence: {e}")
            return None

    def validate_action_sequence(self, action_sequence):
        if not isinstance(action_sequence, ActionSequence):
            rospy.logerr("Planning Agent: Action sequence is not an ActionSequence object")
            return False

        for item in action_sequence.actions:
            if not isinstance(item, RoboticAction):
                rospy.logerr(f"Planning Agent: Invalid item in action sequence: {item}")
                return False

            # Check if all actions in the sequence are valid
            invalid_actions = []
            for action in item.planning_sequence:
                action_name = action.split('(')[0]  # Extract action name
                if not any(action_name == valid for valid in self.robot_actions.keys()):
                    invalid_actions.append(action)

            if invalid_actions:
                rospy.logerr(f"Planning Agent: Invalid actions in sequence: {', '.join(invalid_actions)}")
                return False

        rospy.loginfo("Planning Agent: Action sequence validated successfully")
        return True

    def execute_preliminary_steps(self):
        # Placeholder for executing preliminary steps
        rospy.loginfo("Planning Agent: Executing preliminary steps for safety.")

    def write_json_file(self, action_sequence):
        # Write the action sequence to a JSON file
        robot_sequence_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'robot_sequence')
        os.makedirs(robot_sequence_dir, exist_ok=True)
        timestamp = rospy.Time.now().to_sec()
        json_file_name = f"action_sequence_{timestamp}.json"
        json_file_path = os.path.join(robot_sequence_dir, json_file_name)
        
        with open(json_file_path, 'w') as json_file:
            json.dump(action_sequence.model_dump(), json_file, indent=4)
        rospy.loginfo(f"Planning Agent: Action sequence JSON file created at {json_file_path}")
        
        # Print the contents of the JSON file
        with open(json_file_path, 'r') as json_file:
            rospy.loginfo(f"Planning Agent: JSON file contents:\n{json_file.read()}")
        
        return json_file_path

    def execute_actions(self, action_sequence):
        # Publish actions to the robot control interface
        try:
            for action in action_sequence.actions:
                for step in action.planning_sequence:
                    self.robot_control_pub.publish(step)
                    rospy.loginfo(f"Planning Agent: Executed action: {step}")
            return True, "Action sequence executed successfully."
        except Exception as e:
            rospy.logerr(f"Planning Agent: Error executing actions: {e}")
            return False, f"Error executing actions: {e}"

if __name__ == '__main__':
    try:
        planning_agent = PlanningAgent()
        rospy.loginfo("Planning Agent: Ready to receive plan execution requests.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
