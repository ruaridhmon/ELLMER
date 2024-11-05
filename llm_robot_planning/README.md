# LLM Robot Planning

This project contains two main subfolders:

1. `custom_gpt_examples`: This subfolder includes examples demonstrating how to configure a custom GPT-4 model to control a robot arm, as presented in our paper. It also contains the server files required for the custom GPT-4 to interact with ROS through a web interface.

2. `haystack_rag_pipelines`: This subfolder provides examples for the open-source approach, featuring code to evaluate our method against non-RAG-based approaches and showcasing flexible RAG-LLM integration using Haystack with different document stores, such as ElasticSearch.