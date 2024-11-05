import os
from haystack import Pipeline, Document
from haystack.document_stores.in_memory import InMemoryDocumentStore
from haystack.components.builders import PromptBuilder
from haystack.components.generators import OpenAIGenerator
import json
from pathlib import Path
from haystack.components.retrievers.in_memory import InMemoryBM25Retriever
import asyncio
from haystack_integrations.components.evaluators.ragas import RagasEvaluator, RagasMetric
from haystack.components.evaluators import FaithfulnessEvaluator
from haystack.components.evaluators import ContextRelevanceEvaluator
from haystack.components.generators import HuggingFaceAPIGenerator
from haystack.utils import Secret
from haystack import Document
import gc

NUMBER_OF_QUESTIONS_TO_EVALUATE = 5
GENERATE_AND_SAVE_RESPONSES = False

# POSSIBLE LLM NAMES: zephyr-7b-beta, gpt-4o-mini, gpt-3.5-turbo, gpt-4
LLM_name = "gpt-4"

if LLM_name in ["gpt-4o-mini", "gpt-3.5-turbo", "gpt-4"]:
    os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY") or input("Please enter your OpenAI API key: ")

hf_token = os.getenv("HUGGINGFACE_API_TOKEN") or input("Please enter your Hugging Face API token: ")


def get_llm_rag(LLM_name):

    if LLM_name == "zephyr-7b-beta":
        llm_rag = HuggingFaceAPIGenerator(api_type="serverless_inference_api",
                                        api_params={"model": "HuggingFaceH4/zephyr-7b-beta",
                                                    },
                                        token=Secret.from_token(hf_token))

    if LLM_name in ["gpt-4o-mini", "gpt-3.5-turbo", "gpt-4"]:
        llm_rag = OpenAIGenerator(model=LLM_name)

    return llm_rag



def get_llm_no_rag(LLM_name):

    if LLM_name == "zephyr-7b-beta":
        llm_no_rag = HuggingFaceAPIGenerator(api_type="serverless_inference_api",
                                             api_params={"model": "HuggingFaceH4/zephyr-7b-beta",
                                                         },
                                            token=Secret.from_token(hf_token))


    if LLM_name in ["gpt-4o-mini", "gpt-3.5-turbo", "gpt-4"]:
        llm_no_rag = OpenAIGenerator(model=LLM_name)

    return llm_no_rag
        

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

eval_pipeline = Pipeline()
faithfulness_evaluator = FaithfulnessEvaluator()

eval_pipeline.add_component("faithfulness_evaluator", faithfulness_evaluator)


def load_from_json(file_path="generated_questions.json"):
    json_path = script_dir / file_path
    with open(json_path, "r") as json_file:
        data = json.load(json_file)
        questions = data["commands"]
    return questions

questions = load_from_json("generated_questions.json")
questions = questions[:NUMBER_OF_QUESTIONS_TO_EVALUATE]


def init_rag_pipeline():
    documents = [Document(content=robot_document_content)] 
    document_store = InMemoryDocumentStore()
    document_store.write_documents(documents)
    retriever_rag = InMemoryBM25Retriever(document_store=document_store)
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


def init_no_rag_pipeline():
    prompt_builder_no_rag = PromptBuilder(template=prompt_template)
    llm_no_rag = get_llm_no_rag(LLM_name)
    no_rag_pipeline = Pipeline()
    no_rag_pipeline.add_component("prompt_builder", prompt_builder_no_rag)
    no_rag_pipeline.add_component("llm", llm_no_rag)
    no_rag_pipeline.connect("prompt_builder", "llm")
    return no_rag_pipeline

def get_responses_rag(questions):
    responses = []

    for count, question in enumerate(questions):
        rag_pipeline = init_rag_pipeline()

        result = rag_pipeline.run({
            "retriever": {"query": question},
            "prompt_builder": {"question": question}
        })

        response = result["llm"]["replies"][0]
        responses.append(response)

        gc.collect()

    return responses


def get_responses_no_rag(questions):
    responses = []
    for count, question in enumerate(questions):

        rag_pipeline = init_no_rag_pipeline()

        result = rag_pipeline.run({
            "prompt_builder": {"question": question}
        })

        response = result["llm"]["replies"][0]
        responses.append(response)

        gc.collect()

    return responses


responses_rag_output_path = script_dir / "responses" / f"responses_rag_{LLM_name}.json"
responses_no_rag_output_path = script_dir / "responses" / f"responses_no_rag_{LLM_name}.json"

if GENERATE_AND_SAVE_RESPONSES:
    responses_rag = get_responses_rag(questions)

    with open(responses_rag_output_path, "w") as file:
        json.dump(responses_rag, file, indent=4)

    responses_no_rag = get_responses_no_rag(questions)

    with open(responses_no_rag_output_path, "w") as file:
        json.dump(responses_no_rag, file, indent=4)

else: 

    with open(responses_rag_output_path, "r") as file:
        responses_rag = json.load(file)

    with open(responses_no_rag_output_path, "r") as file:
        responses_no_rag = json.load(file)


CONTEXTS = [[doc.content for doc in documents]] * len(questions)

responses_rag = responses_rag[:NUMBER_OF_QUESTIONS_TO_EVALUATE]
responses_no_rag = responses_no_rag[:NUMBER_OF_QUESTIONS_TO_EVALUATE]

async def evaluate_async():
    results_no_rag = eval_pipeline.run({
        "faithfulness_evaluator": {"questions": questions, "contexts": CONTEXTS, "predicted_answers": responses_no_rag},
    })

    results_rag = eval_pipeline.run({
        "faithfulness_evaluator": {"questions": questions, "contexts": CONTEXTS, "predicted_answers": responses_rag},
    })

    return results_no_rag, results_rag

results_no_rag, results_rag = asyncio.run(evaluate_async())

def calculate_average_score(results, metric_name):
    scores = []
    for result in results['results']:
        for metric in result:
            if metric['name'] == metric_name:
                scores.append(metric['score'])
    
    average_score = sum(scores) / len(scores) if scores else 0
    return average_score

average_faithfulness_no_rag = results_no_rag['faithfulness_evaluator']['score']
print(f"Average Faithfulness (No-RAG): {average_faithfulness_no_rag}")

average_faithfulness_rag = results_rag['faithfulness_evaluator']['score']
print(f"Average Faithfulness (RAG): {average_faithfulness_rag}")