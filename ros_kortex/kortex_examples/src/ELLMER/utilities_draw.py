import requests
import cv2
import matplotlib.pyplot as plt
import os
from openai import OpenAI

api_key = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=api_key)

def return_waypoints(prompt):

    response = client.images.generate(
    model="dall-e-3",
    prompt=prompt,
    size="1024x1024",
    quality="standard",
    n=1,
    )

    generated_image_url = response.data[0].url
    response = requests.get(generated_image_url)

    if response.status_code == 200:
        image_path = os.path.join('generated_images', 'generated_image.png')
        os.makedirs(os.path.dirname(image_path), exist_ok=True)        
        
        with open(image_path, 'wb') as file:
            file.write(response.content)
        print("Image saved successfully.")
        
        # Load the saved image, convert to grayscale, and apply binary threshold
        image = cv2.imread(image_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        
    else:
        print(f"Failed to download the image. Status code: {response.status_code}")

    # this controls how smooth the trace is
    epsilon_factor = 0.001
    epsilon = epsilon_factor * cv2.arcLength(largest_contour, True)
    simplified_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

    x_coords = [point[0][0] for point in simplified_contour]
    y_coords = [point[0][1] for point in simplified_contour]

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    # size of image trace
    target_min, target_max = -0.1, 0.1

    scaled_x_coords = [((x - x_min) / (x_max - x_min)) * (target_max - target_min) + target_min for x in x_coords]
    scaled_y_coords = [ (((y - y_min) / (y_max - y_min)) * (target_max - target_min) + target_min) for y in y_coords]


    waypoints = list(zip(scaled_x_coords, scaled_y_coords))

    return waypoints