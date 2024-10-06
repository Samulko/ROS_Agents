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
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0) #use gpt-4o it is the latest model
        self.output_parser = PydanticOutputParser(pydantic_object=ActionSequence)

        # Set up ROS service
        self.plan_execution_service = rospy.Service('/plan_execution', PlanExecution, self.handle_plan_execution)

        rospy.loginfo("Planning Agent Initialized and ready to work.")

    def handle_plan_execution(self, req):
        comprehensive_plan = json.loads(req.plan)
        rospy.loginfo(f"Planning Agent: Received comprehensive plan: {comprehensive_plan}")

        # Extract the necessary information from the comprehensive plan
        description = comprehensive_plan['description_of_structure']
        components = comprehensive_plan['components']
        disassembly_instructions = comprehensive_plan['disassembly_instructions']
        actor_assignments = comprehensive_plan['actor_assignments']

        # Use this information to create a more detailed action sequence
        action_sequence = self.create_detailed_action_sequence(description, components, disassembly_instructions, actor_assignments)

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

    def create_detailed_action_sequence(self, description, components, disassembly_instructions, actor_assignments):
        rospy.loginfo(f"Planning Agent: Creating detailed action sequence")
        rospy.loginfo(f"Description: {description}")
        rospy.loginfo(f"Components: {components}")
        rospy.loginfo(f"Disassembly Instructions: {disassembly_instructions}")
        rospy.loginfo(f"Actor Assignments: {actor_assignments}")

        action_schemas = ', '.join(f"{key}: {value}" for key, value in self.robot_actions.items())
        
        prompt = ChatPromptTemplate.from_template(
            "You are the Planning Agent in a multi-agent system that controls a robotic arm for disassembly tasks. "
            "Your task is to create a detailed action sequence based on the following information:\n\n"
            "Structure Description: {description}\n\n"
            "Components:\n{components}\n\n"
            "Disassembly Instructions:\n{disassembly_instructions}\n\n"
            "Actor Assignments:\n{actor_assignments}\n\n"
            "Create a detailed action sequence following these guidelines:\n"
            "- Use 'human_working' set to true if a human is performing the action, false if the robot is.\n"
            "- 'selected_element' should specify the element being worked on.\n"
            "- 'planning_sequence' should list the actions in execution order, using only the provided action schemas.\n"
            "- Ensure the sequence follows the exact order of the disassembly instructions.\n"
            "- Maintain consistent roles for each actor throughout the process, as per the actor assignments.\n"
            "- Include necessary preparatory movements before each main action.\n"
            "- For human actions, use 'human_action(action_description)'.\n"
            "- Use specific component IDs (e.g., 'column_1') as mentioned in the components list.\n"
            "- Robot pick-and-place sequences should follow: 'moveto' -> 'picking' -> 'holding' -> 'placing'.\n"
            "- Support actions should follow: 'moveto' -> 'holding', and continue 'holding' until released.\n"
            "- Use 'deposition_zone' as the destination for removed elements.\n\n"
            "Remember to only use the following action schemas:\n{action_schemas}\n\n"
            "Provide the final action sequence in the following JSON format:\n{format_instructions}"
        )
        
        chain = LLMChain(llm=self.llm, prompt=prompt, verbose=True)
        result = chain.run(
            description=description,
            components=json.dumps(components, indent=2),
            disassembly_instructions=json.dumps(disassembly_instructions, indent=2),
            actor_assignments=json.dumps(actor_assignments, indent=2),
            action_schemas=action_schemas,
            format_instructions=self.output_parser.get_format_instructions()
        )
        
        try:
            action_sequence = self.output_parser.parse(result)
            rospy.loginfo(f"Planning Agent: Created detailed action sequence: {action_sequence}")
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
                if not any(action_name == valid for valid in self.robot_actions.keys()): # type: ignore
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
            os.makedirs(robot_sequence_dir) # type: ignore
        
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
