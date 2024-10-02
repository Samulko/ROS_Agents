#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from multi_agent_system.srv import ValidateRequest, ValidateRequestResponse
import openai
from openai import OpenAI
from langchain.prompts import ChatPromptTemplate
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain_community.document_loaders import JSONLoader
from dotenv import load_dotenv
import os
import json

# Load environment variables from .env file
load_dotenv()
print(f"OPENAI_API_KEY loaded: {'Yes' if os.getenv('OPENAI_API_KEY') else 'No'}")

class StructuralEngineerAgent:
    def __init__(self):
        try:
            # Initialize ROS node
            rospy.init_node('structural_engineer_agent', anonymous=False)

            # Get OpenAI API key from environment variables
            self.openai_api_key = os.getenv('OPENAI_API_KEY')
            if not self.openai_api_key:
                raise ValueError("OPENAI_API_KEY not found in environment variables")

            # Initialize OpenAI client
            self.client = OpenAI(api_key=self.openai_api_key)

            # Initialize RAG (Retrieval-Augmented Generation) system
            self.initialize_rag_system()

            # Define the updated prompt template for the AI
            self.prompt = ChatPromptTemplate.from_template("""
                You are the Structural Engineer Agent in a multi-agent robotic system for disassembly tasks. Your role is to validate requests against current disassembly manuals using an RAG system. You are responsible for:

                1. Analyze the request and compare it to the structures in your RAG document.
                2. If a match is found:
                   a. Confirm that the structure matches one in the RAG system.
                   b. Provide a brief description of the matching structure.
                   c. Pass this information along with the information on the proper disassembly sequence as it is described in the RAG to the manager agent for further processing.
                3. If no match is found:
                   a. Clearly communicate to the manager agent that the structure does not match any in the RAG system.
                4. In case of partial matches or ambiguities:
                   a. Explain the nature of the partial match or ambiguity.
                   b. If the match is close to what is described, respond positively and pass the matching description.
                   c. Request more information if needed.

                Current request: {request}
                Relevant information from the manuals: {context}

                AI: Let's analyze this request systematically:

                1. Compare the request to the structures in the RAG system.
                2. Determine if there's a full match, partial match, or no match.
                3. Based on the result:
                   a. For a full match: Confirm the match and pass the information on the proper disassembly sequence as it is described in the RAG to the manager agent for further processing. In this case you must mention the proceedure is standard.
                   b. For no match: Clearly state that no matching structure was found in the RAG system. You must mention the proceedure is not standard.
                   c. For partial matches or ambiguities: Explain the situation and indicate what additional information might be needed. If the match is close but not a full match, pass the closest structure to the manager. You must mention that the proceedure is standard.
                   d. Mention either that the proceedure is standard or not standard.

                Analysis and Validation Result:
            """)

            # Set up ROS service for request validation
            self.validate_request_service = rospy.Service('/validate_request', ValidateRequest, self.handle_validate_request)
            self.structural_engineer_feedback_pub = rospy.Publisher('/structural_engineer_feedback', String, queue_size=10)

            rospy.loginfo("Structural Engineer Agent Initialized.")
        except Exception as e:
            rospy.logerr(f"Error initializing Structural Engineer Agent: {str(e)}")
            raise


    def initialize_rag_system(self):
        try:
            # Initialize the RAG system by loading and processing disassembly manuals
            manual_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'disassembly_manuals.json')
            rospy.loginfo(f"Manual path: {manual_path}")
            rospy.loginfo(f"File exists: {os.path.exists(manual_path)}")
            
            if not os.path.exists(manual_path):
                raise FileNotFoundError(f"Disassembly manual not found at {manual_path}")

            def extract_data(data):
                # Extract relevant information from the JSON data
                return ' '.join([
                    data['name'],
                    ' '.join(data['description_of_structure']),
                    ' '.join(data['components']),
                    ' '.join(data['disassembly_instructions']),
                    ' '.join(data['safety_notes'])
                ])

            # Load JSON data using Langchain's JSONLoader
            loader = JSONLoader(
                file_path=manual_path,
                jq_schema='.procedures[]',
                content_key=None,
                text_content=False  # Set this to False to handle non-string content
            )

            # Process the loaded documents
            documents = loader.load()
            rospy.loginfo(f"Loaded {len(documents)} documents")

            text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=50)
            texts = text_splitter.split_documents(documents)

            # Create a vector store for efficient similarity search
            embeddings = OpenAIEmbeddings()
            self.vectorstore = FAISS.from_documents(texts, embeddings)
            rospy.loginfo("RAG system initialized successfully")
        except Exception as e:
            rospy.logerr(f"Error initializing RAG system: {str(e)}")
            raise

    def generate_disassembly_plan(self, structure_info):
        prompt = f"""
        Given the following structure information:
        {structure_info}

        Generate a high-level disassembly plan. The plan should be a list of steps, each describing a major disassembly action.
        Consider the structure's components and general disassembly instructions.

        Format your response as a list of steps, each on a new line, starting with a number.
        """

        response = self.client.chat.completions.create(
            temperature=0.6,
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a structural engineer specializing in disassembly procedures."},
                {"role": "user", "content": prompt}
            ]
        )
        
        disassembly_plan = response.choices[0].message.content if response.choices else ""
        return disassembly_plan

    def handle_validate_request(self, req):
        try:
            # Handle incoming validation requests
            request = req.request
            rospy.loginfo(f"StructuralEngineerAgent: Validating request: {request}")

            # Retrieve relevant context from the RAG system
            docs = self.vectorstore.similarity_search(request, k=2)
            context = "\n".join([doc.page_content for doc in docs])
            rospy.loginfo(f"StructuralEngineerAgent: Retrieved context: {context}")

            # Use the OpenAI client to validate the request
            response = self.client.chat.completions.create(
                temperature=0.6,
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": self.prompt.format(request=request, context=context)},
                    {"role": "user", "content": request}
                ]
            )
            validation_details = response.choices[0].message.content
            rospy.loginfo(f"StructuralEngineerAgent: Validation details: {validation_details}")
            self.structural_engineer_feedback_pub.publish(f"Validation result: {validation_details}")

            # Determine if the request follows standard procedures
            is_standard = "standard" in validation_details.lower() if validation_details else False and "not standard" not in validation_details.lower() if validation_details else True

            # Generate disassembly plan
            disassembly_plan = response.choices[0].message.content if response.choices else ""
            rospy.loginfo(f"StructuralEngineerAgent: Is standard: {is_standard}")
            return ValidateRequestResponse(is_standard=is_standard, validation_details=validation_details, disassembly_plan=disassembly_plan)
        except Exception as e:
            rospy.logerr(f"StructuralEngineerAgent: Error handling validate request: {str(e)}")
            return ValidateRequestResponse(is_standard=False, validation_details=f"Error: {str(e)}", disassembly_plan="")

if __name__ == '__main__':
    try:
        structural_engineer_agent = StructuralEngineerAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Structural Engineer Agent failed: {str(e)}")
