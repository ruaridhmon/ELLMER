openapi: "3.1.0"
info:
  title: "Action Queue Service"
  description: "A simple service for queuing and retrieving predefined actions."
  version: "v1.0.0"
servers:
  - url: "https://kinovaapi.com"
paths:
  /send_action:
    post:
      summary: "Enqueue an action"
      description: "Adds an action to the queue with its metadata."
      operationId: "enqueueAction"
      x-openai-isConsequential: false  # Ensure no permission is required
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                action:
                  type: object
                  properties:
                    type:
                      type: string
                      description: "The type of action to be performed."
                      example: "run_code"
                    payload:
                      type: object
                      properties:
                        parameters:
                          type: object
                          description: "The parameters needed to perform the action."
                          example: "speed = 0.05\\nself.robot.publish_twist_nosleep([0, 0, -speed, 0, 0, 0])\\nrospy.sleep(1)\\nself.robot.publish_twist_nosleep([0, 0, 0, 0, 0, 0])"
                    metadata:
                      type: object
                      properties:
                        timestamp:
                          type: string
                          format: "date-time"
                          description: "The time the action was queued."
                        priority:
                          type: string
                          description: "The priority of the action."
                          example: "high"
                        source:
                          type: string
                          description: "The source of the action."
                          example: "web_interface"
              required:
                - type
                - payload
      responses:
        '200':
          description: "Action successfully added to the queue."
          content:
            application/json:
              schema:
                type: object
                properties:
                  status:
                    type: string
                    example: "success"

  /get_action:
    get:
      summary: "Dequeue an action"
      description: "Retrieves and removes the first action from the queue."
      operationId: "dequeueAction"
      x-openai-isConsequential: false  # Ensure no permission is required
      responses:
        '200':
          description: "Action successfully retrieved from the queue."
          content:
            application/json:
              schema:
                type: object
                properties:
                  action:
                    type: object
                    properties:
                      type:
                        type: string
                        description: "The type of the dequeued action."
                        example: "move_arm"
                      payload:
                        type: object
                        properties:
                          code:
                            type: string
                            description: "The code to be executed."
                            example: "speed = 0.05\\nself.robot.publish_twist_nosleep([0, 0, -speed, 0, 0, 0])\\nrospy.sleep(1)\\nself.robot.publish_twist_nosleep([0, 0, 0, 0, 0, 0])"
                      metadata:
                        type: object
                        properties:
                          timestamp:
                            type: string
                            format: "date-time"
                            description: "The time the action was queued."
                          priority:
                            type: string
                            description: "The priority of the action."
                            example: "high"
                          source:
                            type: string
                            description: "The source of the action."
                            example: "web_interface"
        '204':
          description: "No actions in the queue."