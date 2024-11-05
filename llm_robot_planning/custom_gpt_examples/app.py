
### This file is run in the AWS server so that the custom GPT can send code and communicate with the robot ###
from flask import Flask, request, jsonify
import queue
import time
import logging
from flask_cors import CORS  # Import CORS

app = Flask(__name__)

CORS(app, origins=[f"http://localhost:{5000}", "https://chat.openai.com"])

# Initialize a queue to hold actions
actions_queue = queue.Queue()

logging.basicConfig(level=logging.DEBUG)

@app.route('/send_action', methods=['POST'])
def receive_action():
    logging.debug(f"Simple POST received with data: {request.json}")
    # Retrieve the JSON from the request
    action = request.json
    print(f'action {action}')
    # Validate the action structure
    if 'action' in action and 'type' in action['action'] and 'payload' in action['action']:
        # Add the action to the queue
        actions_queue.put(action)
        return jsonify({"status": "success","action_feedback": "good"}), 200
    else:
        return jsonify({"status": "error", "message": "Invalid action format"}), 400


@app.route('/get_action', methods=['GET'])
def give_action():
    try:
        # Wait for an action to be available and then return it
        action = actions_queue.get(timeout=30)  # Adjust timeout as needed
        return jsonify(action), 200
    except queue.Empty:
        # Return a 204 No Content status if no action is available within the timeout
        return jsonify({}), 204

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)