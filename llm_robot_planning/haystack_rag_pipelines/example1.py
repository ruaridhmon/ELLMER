import os
from haystack import Pipeline, Document
from haystack.document_stores.in_memory import InMemoryDocumentStore
from haystack.dataclasses import ChatMessage
import gradio as gr
from pathlib import Path
from haystack import Pipeline, PredefinedPipeline

os.environ["OPENAI_API_KEY"] = input("Please enter your OpenAI API key: ")

script_dir = Path(__file__).parent
robot_document_path = script_dir / "custom_knowledge.md"

with open(robot_document_path, 'r') as f:
    robot_document_content = f.read()

documents = [Document(content=robot_document_content)] 
document_store = InMemoryDocumentStore()

indexing_pipeline =  Pipeline.from_template(PredefinedPipeline.INDEXING)
indexing_pipeline.run(data={"sources": [robot_document_path]})
rag_pipeline =  Pipeline.from_template(PredefinedPipeline.RAG)


def rag_pipeline_func(query: str):
    result = rag_pipeline.run(data={"prompt_builder": {"query":query}, "text_embedder": {"text": query}})

    return {"reply": result["llm"]["replies"][0]}


messages = [
    ChatMessage.from_system(
        "Don't make assumptions about what values to plug into functions. Ask for clarification if a user request is ambiguous."
    )
]

def chatbot_with_fc(message, history):
    messages.append(ChatMessage.from_user(message))
    print(f'messages {messages}')
    response = rag_pipeline_func(message)
    print(f'response {response}')

    messages.append(ChatMessage.from_assistant(response['reply']))

    return response['reply']

demo = gr.ChatInterface(
    fn=chatbot_with_fc,
    examples=[
        "Move up 5 seconds",
        "Move left then move up 3 seconds",
        "Pour 10 grams of water",
    ],
    title="Ask the Kinova to do anything!",
)

demo.launch()