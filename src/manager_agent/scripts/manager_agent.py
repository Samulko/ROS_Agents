#!/usr/bin/env python3

# Last commit: 8763551 - changes to system prompts

import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, ValidateRequestResponse, StabilityAnalysis, StabilityAnalysisResponse, PlanExecution, PlanExecutionResponse
import openai
from openai import OpenAI
from langchain_openai import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain.prompts import ChatPromptTemplate
from langchain.chains import LLMChain
from dotenv import load_dotenv
import os
import sys

# Load environment variables from .env file
load_dotenv()
print(f"OPENAI_API_KEY loaded in Manager Agent: {'Yes' if os.getenv('OPENAI_API_KEY') else 'No'}")

class ManagerAgent:
    def __init__(self):
        try:
            # Initialize ROS node
            rospy.init_node('manager_agent', anonymous=False, log_level=rospy.DEBUG)
            rospy.loginfo("ROS node initialized successfully")

            # Get OpenAI API key from environment variables
            self.openai_api_key = os.getenv('OPENAI_API_KEY')
            if not self.openai_api_key:
                rospy.logerr("OPENAI_API_KEY not found in environment variables")
                raise ValueError("OPENAI_API_KEY not set")
            rospy.loginfo("OpenAI API key loaded successfully")
            
            # Initialize ChatOpenAI model
            self.llm = ChatOpenAI(temperature=0.6, model_name="gpt-4o-mini", openai_api_key=self.openai_api_key)

            # Initialize conversation memory
            self.memory = ConversationBufferMemory(return_messages=True)

            # Define the prompt template for the AI
            self.prompt = ChatPromptTemplate.from_template(
                """
                You are the Manager Agent in a multi-agent robotic system for industrial disassembly tasks. Your role is to coordinate all interactions and assign tasks to other agents, ensuring safe, efficient, and compliant operations. You are responsible for:

                1. Interpreting user commands accurately, even if they are ambiguous or complex. Ask for clarification if you don't understand, or the input is outside your scope.
                2. Maintaining context over multiple interactions using conversation history.
                3. Prioritizing and coordinating tasks among specialized agents.
                4. Synthesizing information from various agents to make informed decisions.
                5. Communicating results, progress, and any issues to the user clearly and proactively.
                6. Continuously monitoring task progress and adjusting plans as needed. If you are not sure, ask the user for help.

                The agents you coordinate are:
                - Structural Engineer Agent: Validates requests against current disassembly manuals using a RAG system.
                - Stability Agent: Analyzes the stability of structures during disassembly and suggests safety measures.
                - Planning Agent: Generates detailed action sequence plans for industrial arm robot control systems.
                - Safety Agent: Oversees all operations to ensure compliance with safety standards and regulations.

                Use your advanced natural language understanding to interpret the user's intent and maintain conversation context. When processing a request:

                1. Interpret the user's intent and clarify if necessary.
                2. Determine which agent(s) need to be involved.
                3. Prioritize the task within the current workflow.
                4. Coordinate the necessary information flow between agents.
                5. Synthesize the results from various agents.
                6. Develop a primary plan and contingency plans.
                7. Monitor progress and provide regular updates to the user.

                Always strive for clear communication, efficient task routing, and safe operation. If any stage cannot be completed safely or efficiently, communicate this to the user along with the reasons and possible alternatives.

                Current conversation:
                {history}
                Human: {human_input}
                AI: Let's process this request step by step:
                1. Interpret the user's intent.
                2. Determine which agent(s) need to be involved.
                3. Prioritize the task and coordinate information flow.
                4. Synthesize results and develop plans.
                5. Formulate a clear response or action plan for the user. Be brief in your response.
                Response:
                """
            )

            # Initialize service proxies to None
            self.validate_request = None
            self.stability_analysis = None
            self.plan_execution = None

            # Subscribe to the /user_command topic
            self.user_command_sub = rospy.Subscriber('/user_command', String, self.handle_user_command)

            # Publishers for user feedback and agent responses
            self.user_feedback_pub = rospy.Publisher('/user_feedback', String, queue_size=10)
            self.manager_response_pub = rospy.Publisher('/manager_response', String, queue_size=10)

            # Try to connect to the Structural Engineer Agent and Stability Agent services
            rospy.Timer(rospy.Duration(1), self.try_connect_services)

            rospy.loginfo("Manager Agent initialized successfully.")
        except rospy.ROSInitException as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"Error initializing Manager Agent: {e}")
            raise

    def try_connect_services(self, event):
        # Attempt to connect to the Structural Engineer Agent service
        if self.validate_request is None:
            try:
                rospy.wait_for_service('/validate_request', timeout=1)
                self.validate_request = rospy.ServiceProxy('/validate_request', ValidateRequest)
                rospy.loginfo("Connected to /validate_request service")
            except rospy.ROSException:
                rospy.logwarn("Waiting for /validate_request service...")
            except rospy.ROSInterruptException:
                rospy.logwarn("ROS master is not running. Unable to connect to /validate_request service.")

        # Attempt to connect to the Stability Agent service
        if self.stability_analysis is None:
            try:
                rospy.wait_for_service('/stability_analysis', timeout=1)
                self.stability_analysis = rospy.ServiceProxy('/stability_analysis', StabilityAnalysis)
                rospy.loginfo("Connected to /stability_analysis service")
            except rospy.ROSException:
                rospy.logwarn("Waiting for /stability_analysis service...")
            except rospy.ROSInterruptException:
                rospy.logwarn("ROS master is not running. Unable to connect to /stability_analysis service.")

        # Attempt to connect to the Planning Agent service
        if self.plan_execution is None:
            try:
                rospy.wait_for_service('/plan_execution', timeout=1)
                self.plan_execution = rospy.ServiceProxy('/plan_execution', PlanExecution)
                rospy.loginfo("Connected to /plan_execution service")
                self.user_feedback_pub.publish("Planning Agent is connected and ready to work.")
            except rospy.ROSException:
                rospy.logwarn("Waiting for /plan_execution service...")
            except rospy.ROSInterruptException:
                rospy.logwarn("ROS master is not running. Unable to connect to /plan_execution service.")

        # Check if ROS master is running
        if rospy.is_shutdown():
            rospy.logerr("ROS master is not running. Unable to initialize Manager Agent properly.")
            self.user_feedback_pub.publish("Error: ROS master is not running. Please start roscore and try again.")

    def handle_user_command(self, msg):
        try:
            # Process incoming user commands
            user_command = msg.data
            rospy.loginfo(f"[ManagerAgent] Received user command: {user_command}")

            # Always publish an initial acknowledgment
            self.user_feedback_pub.publish(f"Received command: {user_command}. Processing...")
            self.manager_response_pub.publish(f"Processing command: {user_command}")

            # Use the language model to interpret the command
            try:
                rospy.loginfo(f"[ManagerAgent] Invoking language model for command: {user_command}")
                response = self.llm.invoke(input=f"Interpret this command for disassembling a simple portal frame: {user_command}")
                interpreted_command = response.content.strip()
                rospy.loginfo(f"[ManagerAgent] Interpreted command: {interpreted_command}")

                # Process the interpreted command
                self.process_command(interpreted_command)
            except Exception as e:
                rospy.logerr(f"[ManagerAgent] Error interpreting command: {e}")
                self.user_feedback_pub.publish(f"Error interpreting command: {e}. Please try again.")
        except rospy.ROSInterruptException:
            rospy.logerr("ROS master is not running. Unable to process user command.")
            self.user_feedback_pub.publish("Error: ROS master is not running. Please start roscore and try again.")
        except Exception as e:
            rospy.logerr(f"[ManagerAgent] Unexpected error handling user command: {e}")
            self.user_feedback_pub.publish(f"An unexpected error occurred. Please try again later.")
        finally:
            rospy.loginfo("[ManagerAgent] Finished processing user command")

    def process_command(self, command):
        # Process the interpreted command and interact with other agents
        if command.lower() == "test system":
            self.run_system_test()
            return

        if self.validate_request is None:
            rospy.logwarn("[ManagerAgent] Structural Engineer Agent service is not available. Retrying connection...")
            self.user_feedback_pub.publish("I'm sorry, but I can't validate your request at the moment. Please try again later.")
            return

        try:
            # Validate the request with Structural Engineer Agent
            validation_response = self.validate_request(command)
            disassembly_plan = validation_response.disassembly_plan

            if validation_response.is_standard:
                rospy.loginfo(f"[ManagerAgent] Request is standard: {validation_response.validation_details}")
                self.user_feedback_pub.publish(f"Your request follows standard procedures.")
                
                # Proceed directly to planning for standard requests
                if self.plan_execution is None:
                    rospy.logwarn("[ManagerAgent] Planning Agent service is not available.")
                    self.user_feedback_pub.publish("I'm sorry, but I can't perform planning at the moment. Please try again later.")
                    return
                
                try:
                    planning_response = self.plan_execution(disassembly_plan)
                    if planning_response.success:
                        self.user_feedback_pub.publish(f"Planning complete. Execution details: {planning_response.execution_details}")
                    else:
                        self.user_feedback_pub.publish(f"Planning failed. Details: {planning_response.execution_details}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[ManagerAgent] Planning service call failed: {e}")
                    self.user_feedback_pub.publish(f"I encountered an error during planning. Please try again.")
                except Exception as e:
                    rospy.logerr(f"[ManagerAgent] Unexpected error during planning: {e}")
                    self.user_feedback_pub.publish(f"An unexpected error occurred during planning. Please try again later.")
            else:
                rospy.loginfo(f"[ManagerAgent] Request is non-standard: {validation_response.validation_details}")
                self.user_feedback_pub.publish(f"Your request doesn't follow standard procedures. Proceeding with stability analysis.")

                # Perform stability analysis for non-standard requests
                if self.stability_analysis is None:
                    rospy.logwarn("[ManagerAgent] Stability Agent service is not available.")
                    self.user_feedback_pub.publish("I'm sorry, but I can't perform stability analysis at the moment. Please try again later.")
                    return
                
                try:
                    stability_response = self.stability_analysis(command)
                    stability_aware_plan = stability_response.stability_aware_plan
                    self.user_feedback_pub.publish(f"Stability analysis complete. {'The task is safe to execute.' if stability_response.is_safe else f'The task requires modifications: {stability_response.modifications}'}")

                    # Add the stability result to the conversation memory
                    self.memory.chat_memory.add_ai_message(f"Stability analysis result: {'Safe' if stability_response.is_safe else 'Unsafe. Modifications required: ' + stability_response.modifications}")

                    # Proceed with planning using the stability-aware plan
                    if self.plan_execution is None:
                        rospy.logwarn("[ManagerAgent] Planning Agent service is not available.")
                        self.user_feedback_pub.publish("I'm sorry, but I can't perform planning at the moment. Please try again later.")
                        return
                    
                    try:
                        planning_response = self.plan_execution(stability_aware_plan)
                        if planning_response.success:
                            self.user_feedback_pub.publish(f"Planning complete. Execution details: {planning_response.execution_details}")
                        else:
                            self.user_feedback_pub.publish(f"Planning failed. Details: {planning_response.execution_details}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"[ManagerAgent] Planning service call failed: {e}")
                        self.user_feedback_pub.publish(f"I encountered an error during planning. Please try again.")
                    except Exception as e:
                        rospy.logerr(f"[ManagerAgent] Unexpected error during planning: {e}")
                        self.user_feedback_pub.publish(f"An unexpected error occurred during planning. Please try again later.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[ManagerAgent] Stability analysis service call failed: {e}")
                    self.user_feedback_pub.publish(f"I encountered an error during stability analysis. Please try again.")
                except Exception as e:
                    rospy.logerr(f"[ManagerAgent] Unexpected error during stability analysis: {e}")
                    self.user_feedback_pub.publish(f"An unexpected error occurred during stability analysis. Please try again later.")

            # Add the validation result to the conversation memory
            self.memory.chat_memory.add_ai_message(f"Validation result: {validation_response.validation_details}")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"[ManagerAgent] Service call failed: {e}")
            self.user_feedback_pub.publish(f"I encountered an error while processing your request. Please try again.")

    def run_system_test(self):
        rospy.loginfo("[ManagerAgent] Running system test...")
        self.user_feedback_pub.publish("Running system test...")

        # Test Structural Engineer Agent
        try:
            validation_response = self.validate_request("Test disassembly request")
            self.user_feedback_pub.publish(f"Structural Engineer Agent test result: {validation_response.validation_details}")
        except Exception as e:
            self.user_feedback_pub.publish(f"Structural Engineer Agent test failed: {str(e)}")

        # Test Stability Agent
        try:
            stability_response = self.stability_analysis("Test stability analysis")
            self.user_feedback_pub.publish(f"Stability Agent test result: {'Safe' if stability_response.is_safe else 'Unsafe'}")
        except Exception as e:
            self.user_feedback_pub.publish(f"Stability Agent test failed: {str(e)}")

        self.user_feedback_pub.publish("System test completed.")

if __name__ == '__main__':
    try:
        manager_agent = ManagerAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass