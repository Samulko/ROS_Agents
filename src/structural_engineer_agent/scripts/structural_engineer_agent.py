#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, ValidateRequestResponse
import openai
from openai import OpenAI
from langchain.prompts import ChatPromptTemplate
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from langchain.text_splitter import CharacterTextSplitter
from langchain_community.document_loaders import JSONLoader
from dotenv import load_dotenv
import os
import json
from datetime import datetime
import instructor
from instructor import OpenAISchema
from typing import List
from pydantic import Field, BaseModel
from langchain.text_splitter import RecursiveCharacterTextSplitter
from tenacity import retry, stop_after_attempt, wait_random_exponential
from langchain.chains import LLMChain
from langchain_core.output_parsers import PydanticOutputParser

# Load environment variables from .env file
load_dotenv()

class Component(BaseModel):
    component_id: str = Field(..., description="Unique identifier for the component.")
    component_type: str = Field(..., description="Type of the component (e.g., beam, column).")
    connections: List[str] = Field(default=[], description="Connected components.")

class DisassemblyStep(BaseModel):
    step: str = Field(..., description="Description of the disassembly step and mention of actor_1 or actor_2")

class ActorAssignment(BaseModel):
    task: str = Field(..., description="Description of the task.")
    actor: str = Field(..., description="Assigned actor (e.g., 'actor_1', 'actor_2').")

class EngineerResponse(OpenAISchema):
    name: str = Field(..., description="Unique identifier for the structure.")
    description_of_structure: str = Field(..., description="Detailed description of the structure.")
    components: List[Component] = Field(..., description="Detailed list of structural components involved in the disassembly.")
    disassembly_instructions: List[DisassemblyStep] = Field(..., description="A numbered list of steps for disassembling the structure.")
    actor_assignments: List[ActorAssignment] = Field(..., description="Assignments of actors to specific tasks or components.")
    safety_instructions: List[str] = Field(..., description="mention the safety instructions for the disassembly process mentioned in the manual")
    user_additional_preferences: str = Field(..., description="User's preferences.")
    is_standard: bool = Field(..., description="Does the structure match any of the structures mentioned in the manual.")
    compliance_references: List[str] = Field(default=[], description="References to standards or manuals.")

class StructuralEngineerAgent:
    example_json = '''
    {
      "name": "Complex Truss Bridge",
      "description_of_structure": "A large-scale truss bridge consisting of multiple interconnected steel beams and joints, forming a triangular pattern for optimal load distribution.",
      "components": [
        {
          "component_id": "top_chord_1",
          "component_type": "beam",
          "connections": ["vertical_1", "diagonal_1", "top_chord_2"]
        },
        {
          "component_id": "bottom_chord_1",
          "component_type": "beam",
          "connections": ["vertical_1", "diagonal_1", "bottom_chord_2"]
        },
        {
          "component_id": "vertical_1",
          "component_type": "column",
          "connections": ["top_chord_1", "bottom_chord_1"]
        },
        {
          "component_id": "diagonal_1",
          "component_type": "beam",
          "connections": ["top_chord_1", "bottom_chord_1"]
        }
      ],
      "disassembly_instructions": [
        {"step": "actor_1 secures the central node where multiple beams connect"},
        {"step": "actor_2 carefully detaches diagonal_1 from top_chord_1 and bottom_chord_1"},
        {"step": "actor_1 removes vertical_1 while actor_2 supports the connected chords"},
        {"step": "actors work together to lower and remove top_chord_1 and bottom_chord_1"}
      ],
      "actor_assignments": [
        {"task": "Secure central node", "actor": "actor_1"},
        {"task": "Detach diagonal beams", "actor": "actor_2"},
        {"task": "Remove vertical columns", "actor": "actor_1"},
        {"task": "Lower and remove chord beams", "actor": "actor_1 and actor_2"}
      ],
      "safety_instructions": [
        "Ensure all load-bearing components are properly supported before removal",
        "Use fall protection equipment when working at heights",
        "Implement a clear communication system between actors to coordinate actions",
        "Regularly inspect tools and equipment for any signs of wear or damage"
      ],
      "user_additional_preferences": "Prioritize the preservation of the diagonal beams for potential reuse.",
      "is_standard": false,
      "compliance_references": ["AASHTO LRFD Bridge Design Specifications", "OSHA Steel Erection Standard"]
    }
    '''

    def __init__(self):
        try:
            # Initialize ROS node
            rospy.init_node('structural_engineer_agent', anonymous=False)

            self.openai_api_key = os.getenv('OPENAI_API_KEY')
            if not self.openai_api_key:
                raise ValueError("OPENAI_API_KEY not found in environment variables")

            # Initialize OpenAI client with Instructor
            self.client = instructor.patch(OpenAI(api_key=self.openai_api_key))

            self.initialize_rag_system()

            # Initialize LangChain components
            self.llm = ChatOpenAI(model="gpt-4o", temperature=0) #use gpt-4o it is the latest model and do not change it!
            self.output_parser = PydanticOutputParser(pydantic_object=EngineerResponse)

            # Set up ROS service for request validation
            self.validate_request_service = rospy.Service('/validate_request', ValidateRequest, self.handle_validate_request)
            self.structural_engineer_feedback_pub = rospy.Publisher('/structural_engineer_feedback', String, queue_size=10)

            # Initialize the prompt template
            self.prompt = ChatPromptTemplate.from_template("""
                You are the Structural Engineer Agent in a multi-agent robotic system for disassembly tasks. Your role is to validate requests against current disassembly manuals using an RAG system. You are responsible for:

                1. Analyze the request and compare it to the structures in your RAG document.
                2. If a match is found:
                   a. Confirm that the structure matches one in the RAG system.
                   b. Provide a brief description of the matching structure.
                   c. Pass this information along with the information on the proper disassembly sequence and safety instructions as they are described in the RAG to the manager agent for further processing.
                3. If no match is found:
                   a. Clearly communicate to the manager agent that the structure does not match any in the RAG system.
                4. In case of partial matches or ambiguities:
                   a. Explain the nature of the partial match or ambiguity.
                   b. If the match is close to what is described, but does not match the number of elements of structural types, respond negatively by saying structure is not standard.
                   c. Request more information if needed.

                Current request: {request}
                Relevant information from the manuals: {context}

                AI: Let's analyze this request systematically:

                1. Compare the request to the structures in the RAG system.
                2. Determine if there's a full match, partial match, or no match.
                3. Based on the result:
                   a. For a full match: Confirm the match and pass the information on the proper disassembly sequence and safety instructions as they are described in the RAG to the manager agent for further processing. In this case you must mention the procedure is standard.
                   b. For no match: Clearly state that no matching structure was found in the RAG system. You must mention the procedure is not standard.
                   c. For partial matches or ambiguities: Explain the situation. If the match is close but not a full match - for instance the structure description does not match the number of elements of structural types, respond negatively by saying structure is not standard.
                   d. Mention either that the procedure is standard or not standard.

                Provide your response in the following JSON format, strictly adhering to the schema:
                {format_instructions}

                Here's an example of the expected JSON format:
                {example_json}

                Ensure that your response includes all required fields, including safety instructions, and follows the exact structure of the schema.

                Analysis and Validation Result:
            """)

            rospy.loginfo("Structural Engineer Agent Initialized.")
        except Exception as e:
            rospy.logerr(f"Error initializing Structural Engineer Agent: {str(e)}")
            raise

    def initialize_rag_system(self):
        try:
            # Update the path to the correct location of the disassembly manual
            manual_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'disassembly_manuals.json')
            rospy.loginfo(f"Manual path: {manual_path}")
            rospy.loginfo(f"File exists: {os.path.exists(manual_path)}")
            
            if not os.path.exists(manual_path):
                raise FileNotFoundError(f"Disassembly manual not found at {manual_path}")

            loader = JSONLoader(
                file_path=manual_path,
                jq_schema='.procedures[]',
                text_content=False
            )

            documents = loader.load()
            rospy.loginfo(f"Loaded {len(documents)} documents")

            @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
            def create_embeddings_with_retry(texts):
                embeddings = OpenAIEmbeddings()
                return embeddings.embed_documents([text.page_content for text in texts])

            embeddings = create_embeddings_with_retry(documents)

            self.vectorstore = FAISS.from_embeddings(
                text_embeddings=list(zip([doc.page_content for doc in documents], embeddings)),
                embedding=OpenAIEmbeddings(),
                metadatas=[doc.metadata for doc in documents]
            )
            rospy.loginfo("RAG system initialized successfully")
        except Exception as e:
            rospy.logerr(f"Error initializing RAG system: {str(e)}")
            raise

    def handle_validate_request(self, req):
        try:
            request = req.request
            rospy.loginfo(f"StructuralEngineerAgent: Validating request: {request}")

            docs = self.vectorstore.similarity_search(request, k=2)
            if not docs:
                return ValidateRequestResponse(is_standard=False, validation_details="No relevant information found in the RAG system.", disassembly_plan="")

            context = "\n".join([doc.page_content for doc in docs])
            rospy.loginfo(f"StructuralEngineerAgent: Retrieved context: {context}")

            response = self.client.chat.completions.create(
                model="gpt-4o", #use gpt-4o it is the latest model
                temperature=0,
                response_model=EngineerResponse,
                messages=[
                    {"role": "system", "content": self.prompt.format(
                        request=request, 
                        context=context, 
                        format_instructions=EngineerResponse.schema_json(),
                        example_json=self.example_json
                    )},
                    {"role": "user", "content": request}
                ]
            )

            self.export_response(request, response)

            validation_details = json.dumps(response.dict(), indent=2)
            disassembly_plan = "\n".join([step.step for step in response.disassembly_instructions])

            self.structural_engineer_feedback_pub.publish(f"Validation result: {validation_details}")

            return ValidateRequestResponse(is_standard=response.is_standard, validation_details=validation_details, disassembly_plan=disassembly_plan)
        except Exception as e:
            rospy.logerr(f"StructuralEngineerAgent: Error handling validate request: {str(e)}")
            return ValidateRequestResponse(is_standard=False, validation_details=f"Error: {str(e)}", disassembly_plan="")

    def export_response(self, request, response):
        export_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'engineer_response')
        os.makedirs(export_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"engineer_response_{timestamp}.json"
        filepath = os.path.join(export_dir, filename)

        export_data = {
            "request": request,
            "response": response.dict()
        }

        with open(filepath, 'w') as f:
            json.dump(export_data, f, indent=4)

        rospy.loginfo(f"Response exported to: {filepath}")

if __name__ == '__main__':
    try:
        structural_engineer_agent = StructuralEngineerAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Structural Engineer Agent failed: {str(e)}")