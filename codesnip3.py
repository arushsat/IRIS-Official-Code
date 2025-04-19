import cv2
import numpy as np
from tensorflow.keras.models import load_model

# Load pre-trained model for satellite image classification
model = load_model('satellite_image_model.h5')

def analyze_satellite_image(image_path):
    # Read and preprocess the image
    image = cv2.imread(image_path)
    image = cv2.resize(image, (224, 224))  # Resize to match input size of the model
    image = np.expand_dims(image, axis=0)  # Add batch dimension
    image = image / 255.0  # Normalize image pixels

    # Predict using the model
    prediction = model.predict(image)

    # Return predicted class (e.g., 'Water Body', 'Desert', etc.)
    return np.argmax(prediction)

# Example usage
image_path = "satellite_image.jpg"
predicted_class = analyze_satellite_image(image_path)
print(f"Predicted resource area: {predicted_class}")
