# Hagstack rag pipeline

This guide provides two examples demonstrating how to control a robot arm using Haystack's Retrieval-Augmented Generation (RAG) pipeline through natural language commands.

### Example 1: 
This example uses a predefined RAG pipeline from Haystack to launch a local website. You can input questions and view the generated code that controls the robot arm.

### Example 2:
This example uses ElasticSearch for scalability. The LLM outputs two responses: 
- 'Audio Output': What would be said to the user. 
- 'Robot Output': The command that would be sent to the robot.

#### Prerequisite Command:
To run this example, start ElasticSearch with the following Docker command:

``` bash
docker run -p 9200:9200 -e "discovery.type=single-node" -e "ES_JAVA_OPTS=-Xms1024m -Xmx1024m" -e "xpack.security.enabled=false" elasticsearch:8.11.1
```

### Evaluation

We compared the effectiveness of generating robot plans with and without RAG in the `evaluate_models.py` file. The `generated_questions.py` script creates 80 task-specific questions based on the range of tasks outlined in `custom_gpt_instructions.md` file. The `evaluate_models.py` file then assesses the accuracy of the responses to these questions.