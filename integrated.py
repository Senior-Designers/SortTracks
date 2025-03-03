import time
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite  # Use TFLite runtime for efficiency

# GPIO Pin Setup
TRIG = 23  # Ultrasonic Trigger Pin
ECHO = 24  # Ultrasonic Echo Pin
STEP_PIN = 17
DIR_PIN = 27
ENA_PIN = 22

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Avoid "channel already in use" warnings
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)

# Load TFLite Model
interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

# Get input and output tensor indices
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Picamera2 setup
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (224, 224)}))  # Ensure model-compatible size
picam2.start()

def get_distance():
    """Measures distance using an ultrasonic sensor."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIG, False)

    start_time, stop_time = time.time(), time.time()
    
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to cm
    return distance

def capture_image():
    """Captures an image and converts it to UINT8 for the TFLite model."""
    picam2.capture_file("image.jpg")  # Save image (optional)
    img = cv2.imread("image.jpg")  # Read image
    img = cv2.resize(img, (224, 224))  # Resize for model
    img = img.astype(np.uint8)  # Convert to UINT8 (expected by model)
    img = np.expand_dims(img, axis=0)  # Add batch dimension
    return img

def classify_image(image):
    """Runs inference on an image and returns the predicted label."""
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    predicted_class = np.argmax(output_data)
    return predicted_class

try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 10:  # Object detected within 10 cm
            print("Object detected! Capturing image...")
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            # Perform action based on classification
            if prediction == 0:
                print("Sorting: Plastic")
            elif prediction == 1:
                print("Sorting: Glass")
            elif prediction == 2:
                print("Sorting: Metal")
            else:
                print("Unknown item")

        time.sleep(1)  # Wait before the next detection

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()
    picam2.stop()
