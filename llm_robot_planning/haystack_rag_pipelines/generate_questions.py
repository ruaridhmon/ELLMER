import os
import json
from haystack import Pipeline, Document
from haystack.document_stores.in_memory import InMemoryDocumentStore
from haystack.components.builders import PromptBuilder
from haystack.components.generators import OpenAIGenerator
from pathlib import Path
from haystack.components.builders import PromptBuilder
import re

os.environ["OPENAI_API_KEY"] = input("Please enter your OpenAI API key: ")

script_dir = Path(__file__).parent
robot_document_path = script_dir / "custom_knowledge.md"

with open(robot_document_path, 'r') as f:
    robot_document_content = f.read()

documents = [Document(content=robot_document_content)] 
document_store = InMemoryDocumentStore()
document_store.write_documents(documents)

prompt_template = """
Given the following documents, generate 80 human-like commands. 
These commands should be tasks or instructions that the system can solve based on the document’s content. 
Make sure the commands are actionable and context-specific.
These commands should be direct, concise, and actionable, avoiding unnecessary detail or explanation.

Documents:
{% for doc in documents %}
    {{ doc.content }}
{% endfor %}

Commands:
1.
2.
3.
4.
...
79.
80.
"""

p = Pipeline()
p.add_component(instance=PromptBuilder(template=prompt_template), name="prompt_builder")
p.add_component(instance=OpenAIGenerator(), name="llm")
p.connect("prompt_builder", "llm")
result = p.run({"prompt_builder": {"documents": documents}})

llm_reply = result["llm"]["replies"][0]
script_dir = Path(__file__).resolve().parent


questions = []
for line in llm_reply.split("\n"):
    stripped_line = line.strip()
    match = re.match(r'^\d+\.\s*(.*)', stripped_line)
    if match:
        command_text = match.group(1).strip()
        questions.append(command_text)
    elif stripped_line:
        questions.append(stripped_line)

def save_to_json(  questions, file_path="generated_questions.json"):
    json_path = script_dir / file_path
    with open(json_path, "w") as json_file:
        json.dump({"   questions": questions}, json_file, indent=4)
    print(f"   questions saved to {json_path}")

save_to_json(  questions, "generated_questions.json")