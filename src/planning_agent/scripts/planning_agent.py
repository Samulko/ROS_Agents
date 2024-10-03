#!/usr/bin/env python3
import rospy
import os  # Add this import
from multi_agent_system.srv import PlanExecution, PlanExecutionResponse
from openai import OpenAI
from pydantic import Field
from typing import List
import instructor
from instructor import OpenAISchema
import json
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import PydanticOutputParser
from langchain.chains import LLMChain

# Load environment variables from .env file
from dotenv import load_dotenv
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
        # Initialize ROS node
        rospy.init_node('planning_agent', anonymous=True)
        
        # Initialize OpenAI client with Instructor
        self.client = instructor.patch(OpenAI())

        # Load robot actions from ROS parameters
        self.robot_actions = rospy.get_param('~robot_actions', {
            "move_in_cartesian_path": "move_in_cartesian_path(move_distance_x, move_distance_y, move_distance_z)",
            "moveto": "moveto",
            "picking": "picking",
            "holding": "holding",
            "placing": "placing",
            "human_action": "human_action(action_description)"
        })

        # Initialize LangChain components
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.output_parser = PydanticOutputParser(pydantic_object=ActionSequence)

        # Set up ROS service
        self.plan_execution_service = rospy.Service('/plan_execution', PlanExecution, self.handle_plan_execution)

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

            # Write the JSON file
            json_file_path = self.write_json_file(action_sequence)

            # Execute actions
            success, execution_details = self.execute_actions(action_sequence)
            return PlanExecutionResponse(success=success, execution_details=f"{execution_details}. JSON file created at {json_file_path}")
        else:
            return PlanExecutionResponse(success=False, execution_details="Invalid action sequence. Please check the plan and try again.")

    def translate_plan(self, plan):
        rospy.loginfo(f"Planning Agent: Translating plan: {plan}")
        action_schemas = ', '.join(f"{key}: {value}" for key, value in self.robot_actions.items())
        
        example_json = '''
        {
          "actions": [
            {
              "human_working": false,
              "selected_element": "element_2",
              "planning_sequence": ["moveto", "holding"]
            },
            {
              "human_working": true,
              "selected_element": "element_1",
              "planning_sequence": ["human_action(remove column 1)"]
            }
          ]
        }
        '''
        
        # Step 1: Extract Numbered Instructions and Preferences
        step1_prompt = ChatPromptTemplate.from_template(
            "You are the Planning Agent in a multi-agent system that controls a robotic arm for disassembly tasks. "
            "Your first task is to extract the numbered Disassembly Instructions and any additional actor preferences from the plan below.\n\n"
            "**Plan:**\n{plan}\n\n"
            "Please provide:\n"
            "1. The numbered Disassembly Instructions exactly as they appear.\n"
            "2. Any additional actor preferences or constraints mentioned in the plan."
        )
        
        step1_chain = LLMChain(llm=self.llm, prompt=step1_prompt, verbose=True)
        extraction_result = step1_chain.run(plan=plan)
        
        # Parse the extraction result
        instructions_and_preferences = self.parse_extraction_result(extraction_result)
        
        # Step 2: Interpret Instructions Step by Step
        step2_prompt = ChatPromptTemplate.from_template(
            "Now, interpret each of the numbered Disassembly Instructions step by step. For each instruction, do the following:\n"
            "1. Identify the actor (human or robot) performing the action, prioritizing the stated preferences.\n"
            "2. Determine the specific element being worked on.\n"
            "3. Decide which action schemas are needed to perform this instruction.\n"
            "4. Explain your reasoning, ensuring it aligns with the stated preferences.\n\n"
            "**Numbered Instructions:**\n{numbered_instructions}\n\n"
            "**Additional Preferences:**\n{additional_preferences}\n\n"
            "Remember to only use the following action schemas:\n{action_schemas}\n\n"
            "Proceed step by step, always prioritizing the stated preferences over default assumptions."
        )
        
        step2_chain = LLMChain(llm=self.llm, prompt=step2_prompt, verbose=True)
        interpreted_steps = step2_chain.run(
            numbered_instructions=instructions_and_preferences['instructions'],
            additional_preferences=instructions_and_preferences['preferences'],
            action_schemas=action_schemas
        )
        
        # Step 3: Generate the Action Sequence
        step3_prompt = ChatPromptTemplate.from_template(
            "Based on your interpretations, generate the action sequence in JSON format. Follow these guidelines:\n"
            "- Strictly adhere to the preferences for human-robot task division stated in the initial prompt\n"
            "- Use 'human_working' set to true if a human is performing the action, false if the robot is.\n"
            "- 'selected_element' should specify the element being worked on.\n"
            "- 'planning_sequence' should list the actions in execution order, using only the provided action schemas.\n"
            "- Ensure the sequence follows the exact order of the numbered instructions.\n"
            "- Maintain consistent roles for each actor throughout the process, as per the stated preferences.\n"
            "- Include necessary preparatory movements before each main action.\n"
            "- For human actions, use 'human_action(action_description)'.\n"
            "- Use specific element names (e.g., 'element_1').\n"
            "- Robot pick-and-place sequences should follow: 'moveto' -> 'picking' -> 'holding' -> 'placing'.\n"
            "- Support actions should follow: 'moveto' -> 'holding', and continue 'holding' until released.\n"
            "- Use 'deposition_zone' as the destination for removed elements.\n\n"
            "**Your Interpretations:**\n{interpreted_steps}\n\n"
            "**Additional Preferences:**\n{additional_preferences}\n\n"
            "Provide the final action sequence in the following JSON format:\n{format_instructions}\n\n"
            "Here is an example:\n{example_json}"
        )
        
        step3_chain = LLMChain(llm=self.llm, prompt=step3_prompt, verbose=True)
        final_result = step3_chain.run(
            interpreted_steps=interpreted_steps,
            additional_preferences=instructions_and_preferences['preferences'],
            format_instructions=self.output_parser.get_format_instructions(),
            example_json=example_json
        )
        
        try:
            action_sequence = self.output_parser.parse(final_result)
            rospy.loginfo(f"Planning Agent: Translated plan into action sequence: {action_sequence}")
            return action_sequence
        except Exception as e:
            rospy.logerr(f"Planning Agent: Failed to generate action sequence: {e}")
            return None

    def parse_extraction_result(self, extraction_result):
        # Split the extraction result into instructions and preferences
        parts = extraction_result.split("Additional actor preferences or constraints:")
        instructions = parts[0].strip()
        preferences = parts[1].strip() if len(parts) > 1 else ""
        return {"instructions": instructions, "preferences": preferences}

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
        rospy.loginfo("Planning Agent: Executing preliminary steps for safety.")

    def write_json_file(self, action_sequence):
        robot_sequence_dir = rospy.get_param('~robot_sequence_dir', '/tmp/robot_sequence')
        rospy.loginfo(f"Writing JSON file to directory: {robot_sequence_dir}")
        
        if not os.path.exists(robot_sequence_dir):
            os.makedirs(robot_sequence_dir)
        
        timestamp = rospy.get_time()
        json_file_name = f"action_sequence_{timestamp}.json"
        json_file_path = os.path.join(robot_sequence_dir, json_file_name)
        
        with open(json_file_path, 'w') as json_file:
            json.dump(action_sequence.dict(), json_file, indent=4)
        rospy.loginfo(f"Planning Agent: Action sequence JSON file created at {json_file_path}")
        
        # Print the contents of the JSON file
        with open(json_file_path, 'r') as json_file:
            rospy.loginfo(f"Planning Agent: JSON file contents:\n{json_file.read()}")
        
        return json_file_path

    def execute_actions(self, action_sequence):
        try:
            for action in action_sequence.actions:
                for step in action.planning_sequence:
                    rospy.loginfo(f"Planning Agent: Executed action: {step}")
            return True, "Action sequence executed successfully."
        except Exception as e:
            rospy.logerr(f"Planning Agent: Error executing actions: {e}")
            return False, f"Error executing actions: {e}"

if __name__ == '__main__':
    try:
        planning_agent = PlanningAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass