#!/usr/bin/env python3

import rospy
from multi_agent_system.srv import StabilityAnalysis, StabilityAnalysisResponse
from std_msgs.msg import String
from langchain_openai import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain_community.document_loaders import JSONLoader
from dotenv import load_dotenv
import os
import json

load_dotenv()

class StabilityAgent:
    def __init__(self):
        rospy.init_node('stability_agent', anonymous=False)

        # Parameters
        self.openai_api_key = rospy.get_param('/openai_api_key')

        # Initialize ChatOpenAI
        self.llm = ChatOpenAI(temperature=0, model="gpt-4o", openai_api_key=self.openai_api_key) # type: ignore

        # Initialize RAG system 
        self.initialize_rag_system()

        # Initialize prompt template
        self.prompt = ChatPromptTemplate.from_template("""
        You are the Stability Agent in a multi-agent robotic arm system for disassembly tasks. Your role is to ensure that requested actions won't compromise structural stability. You are responsible for:

        1. Evaluating the impact of non-standard or complex actions on structural stability.
        2. Suggesting additional steps if necessary to maintain stability.
        3. Using advanced predictive capabilities, including physics simulation.

        Current task: {task}
        Relevant information from past assessments: {context}

        AI: Let's analyze this task for stability risks step by step:
        1. Evaluate the structural risks.
        2. Suggest an element that would be safe to remove first.
           a. You can suggest that the robotic arm stabilizes an element while the human disassembles others.
           b. You can ask the user to re-evaluate their choice to focus on a safer approach.
        3. Provide a detailed explanation of your analysis and recommendations, including:
           a. A sequence of how to disassemble the given structure, element by element.
           b. Suggestions for opportunities where one of the agents (robot or human) can support or hold an element while the other removes a different element to guarantee structural stability.

        Stability analysis result:
        """)

        # Service to analyze stability
        self.stability_analysis_service = rospy.Service('/stability_analysis', StabilityAnalysis, self.handle_stability_analysis)
        self.stability_feedback_pub = rospy.Publisher('/stability_feedback', String, queue_size=10)

        rospy.loginfo("Stability Agent Initialized.")

    def initialize_rag_system(self):
        try:
            # Load and process the past stability assessments
            assessments_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'stability_assessments.json')
            os.makedirs(os.path.dirname(assessments_path), exist_ok=True)
            if not os.path.exists(assessments_path):
                with open(assessments_path, 'w') as f:
                    json.dump({"assessments": []}, f)
            
            def extract_data(data):
                return ' '.join([
                    data['task_description'],
                    ' '.join(data['risk_factors']),
                    ' '.join(data['stability_measures']),
                    ' '.join(data['outcome'])
                ])

            loader = JSONLoader(
                file_path=assessments_path,
                jq_schema='.assessments[]',
                content_key=None,
                text_content=extract_data # type: ignore
            )

            documents = loader.load()
            if not documents:
                rospy.logwarn("No documents loaded from stability_assessments.json. Initializing empty vector store.")
                self.vectorstore = FAISS.from_texts([""], OpenAIEmbeddings())
                return

            text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
            texts = text_splitter.split_documents(documents)

            # Create vector store
            embeddings = OpenAIEmbeddings()
            self.vectorstore = FAISS.from_documents(texts, embeddings)
            rospy.loginfo("RAG system initialized successfully")
        except Exception as e:
            rospy.logerr(f"Error initializing RAG system: {str(e)}")
            self.vectorstore = None

    def generate_stability_aware_plan(self, task, stability_analysis):
        prompt = f"""
        Given the following task and stability analysis:
        Task: {task}
        Stability Analysis: {stability_analysis}

        Generate a detailed, stability-aware disassembly plan. The plan should include specific steps for maintaining stability during disassembly, such as which elements to support and in what order to remove components.

        Format your response as a list of steps, each on a new line, starting with a number. Include specific stability considerations for each step where applicable.
        """

        response = self.llm.invoke(prompt)
        stability_aware_plan = response.content if isinstance(response.content, str) else response.content[0]
        return stability_aware_plan

    def handle_stability_analysis(self, req):
        task = req.task
        rospy.loginfo(f"Analyzing stability for task: {task}")

        # Retrieve relevant context from the RAG system
        docs = self.vectorstore.similarity_search(task, k=2) if self.vectorstore else []
        context = "\n".join([doc.page_content for doc in docs])

        # Use the LLM to analyze stability
        response = self.llm.invoke(self.prompt.format(task=task, context=context))
        analysis = response.content if isinstance(response.content, str) else response.content[0]

        # Determine if the task is safe
        analysis_text = analysis if isinstance(analysis, str) else str(analysis)
        is_safe = "safe to execute" in analysis_text.lower()
        modifications = analysis_text if not is_safe else ""

        # Generate stability-aware plan
        stability_aware_plan = self.generate_stability_aware_plan(task, analysis)

        # Only publish feedback if the analysis was actually requested
        if task != "Test stability analysis":
            self.stability_feedback_pub.publish(f"Stability analysis result: {'Safe' if is_safe else 'Unsafe'}. Modifications: {modifications}")
        
        return StabilityAnalysisResponse(is_safe=is_safe, modifications=modifications, stability_aware_plan=stability_aware_plan)

    def run_physics_simulation(self, task):
        # Placeholder for physics simulation
        # In a real implementation, this would interface with a physics engine or Unity simulation
        rospy.loginfo(f"Running physics simulation for task: {task}")
        # Return simulated results
        return "Simulation results: Task appears stable under normal conditions."

    def update_rag_system(self, task, analysis, outcome):
        # Placeholder for updating the RAG system with new successful task completions
        rospy.loginfo(f"Updating RAG system with new assessment: {task}")
        # In a real implementation, this would add the new assessment to the vector store
        # and potentially retrain or update the embeddings

if __name__ == '__main__':
    try:
        stability_agent = StabilityAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
