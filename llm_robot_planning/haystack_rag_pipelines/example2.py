import os
from haystack import Pipeline, Document
from haystack.components.builders import PromptBuilder
from haystack.components.generators import OpenAIGenerator
from pathlib import Path
from haystack.components.generators import HuggingFaceAPIGenerator
from haystack.utils import Secret
from haystack import Document
import gradio as gr
import uuid
from haystack_integrations.document_stores.elasticsearch import ElasticsearchDocumentStore
from haystack_integrations.components.retrievers.elasticsearch import ElasticsearchBM25Retriever
from haystack.dataclasses import ChatMessage

# ElasticSearch: https://docs.haystack.deepset.ai/docs/elasticsearch-document-store
# RUN THIS COMMAND FIRST: docker run -p 9200:9200 -e "discovery.type=single-node" -e "ES_JAVA_OPTS=-Xms1024m -Xmx1024m" -e "xpack.security.enabled=false" elasticsearch:8.11.1

GENERATE_AND_SAVE_RESPONSES = False
# POSSIBLE LLM NAMES: zephyr-7b-beta, gpt-4o-mini, gpt-3.5-turbo, gpt-4
LLM_name = "gpt-4"

if LLM_name in ["gpt-4o-mini", "gpt-3.5-turbo", "gpt-4"]:
    os.environ["OPENAI_API_KEY"] = input("Please enter your OpenAI API key: ")

hf_token = os.getenv("HUGGINGFACE_API_TOKEN") or input("Please enter your Hugging Face API token: ")

def get_llm_rag(LLM_name):

    if LLM_name == "zephyr-7b-beta":
        llm_rag = HuggingFaceAPIGenerator(api_type="serverless_inference_api",
                                        api_params={"model": "HuggingFaceH4/zephyr-7b-beta",
                                                    },
                                        token=Secret.from_token(hf_token))

    if LLM_name in ["gpt-4o-mini", "gpt-3.5-turbo", "gpt-4"]:
        system_prompt = (
            "When responding, please include 'Audio Output:' and 'Robot Output:' as headings in your answer, following the exact format provided."
        )        
        llm_rag = OpenAIGenerator(model=LLM_name, system_prompt=system_prompt)

    return llm_rag

      

script_dir = Path(__file__).parent
robot_document_path = script_dir / "custom_knowledge.md"

with open(robot_document_path, 'r') as f:
    robot_document_content = f.read()

documents = [Document(content=robot_document_content)] 


prompt_template = """
Given the following documents, provide two outputs:

1. Audio Output: A simple sentence describing the action that the robot is going to perform, which can be used for audio synthesis.
2. Robot Output: Provisde a code snippet to control the robot based on the user's query.

Documents:
{% for doc in documents %}
    {{ doc.content }}
{% endfor %}

Question: {{ question }}
Audio Output:
Robot Output:
"""


def init_rag_pipeline():
    unique_id = str(uuid.uuid4())
    document_store = ElasticsearchDocumentStore(hosts = "http://localhost:9200")
    document_store.write_documents([Document(content=robot_document_content, id=unique_id)])
    retriever_rag = ElasticsearchBM25Retriever(document_store=document_store, top_k=1)
    prompt_builder_rag = PromptBuilder(template=prompt_template)
    llm_rag = get_llm_rag(LLM_name)
    rag_pipeline = Pipeline()
    rag_pipeline.add_component("retriever", retriever_rag)
    rag_pipeline.add_component("prompt_builder", prompt_builder_rag)
    rag_pipeline.add_component("llm", llm_rag)
    rag_pipeline.connect("retriever", "prompt_builder.documents")
    rag_pipeline.connect("prompt_builder", "llm")
    rag_pipeline.warm_up()
    return rag_pipeline


def rag_pipeline_func(question):
    rag_pipeline = init_rag_pipeline()
    result = rag_pipeline.run({
        "retriever": {"query": question},
        "prompt_builder": {"question": question}
    })

    reply = result["llm"]["replies"][0]

    try:
        audio_output = reply.split("Audio Output:")[1].split("\nRobot Output:")[0].strip()
        robot_output = reply.split("\nRobot Output:")[1].strip()
    except (IndexError, AttributeError) as e:
        print(f"Error during split: {e}")
        audio_output = "No specific audio output generated."
        robot_output = "No specific robot output generated."

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