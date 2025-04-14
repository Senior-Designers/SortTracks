import time
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

# GPIO Pin Setup
TRIG = 23  # Ultrasonic Trigger Pin
ECHO = 24  # Ultrasonic Echo Pin
STEP_PIN = 17
DIR_PIN = 27
ENA_PIN = 22

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)

# Enable the stepper motor (HIGH = On)
GPIO.output(ENA_PIN, GPIO.HIGH)

# Load TFLite Model
interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

# Get input and output tensor indices
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Picamera2 setup
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (224, 224)}))  # model-compatible size
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
    picam2.capture_file("image_large.jpg")  # Save image 
    img = cv2.imread("image_large.jpg")  # Read image
    img = cv2.resize(img, (224, 224))  # Resize for model
    img = img.astype(np.uint8)  # Convert to UINT8 (expected by model)
    img = np.expand_dims(img, axis=0)  # Add batch dimension

    # Save the captured image without displaying it
    filename = "image_large.jpg"
    cv2.imwrite(filename, img[0])
    print(f"Image saved as {filename}")

    return img

def classify_image(image):
    """Runs inference on an image and returns the predicted label."""
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    confidence = np.max(output_data)
    predicted_class = np.argmax(output_data)
    if confidence < 0.5:
      predicted_class = 3 # Confidence too low (below 50%), select none
    return predicted_class

def rotate_stepper_motor_1(steps, direction):
    """Rotates the stepper motor by a specified number of steps."""
    GPIO.output(DIR_PIN, direction)  # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.0001)  # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.0001)

try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 30.5:  # Object detected within 1 foot, take picture
            print("Object detected! Capturing image...")
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            # Perform action based on classification and rotate the first motor accordingly
            if prediction == 0:
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, direction=GPIO.HIGH) # Plastic -> push left = 180 clockwise (top motor)
            elif prediction == 1:
                print("Sorting: Aluminum")
                rotate_stepper_motor_1(1600, direction=GPIO.LOW) # Aluminum -> push right = 180 counterclockwise (top motor)
            elif prediction == 2:
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, direction=GPIO.LOW) # Glass -> push right = 180 counterclockwise (top motor)
            elif prediction == 3:
                print("Unknown Item")
                rotate_stepper_motor_1(1600, direction=GPIO.HIGH) # None -> push left = 180 clockwise (top motor)

        time.sleep(5)  # Wait 5 seconds before the next detection

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.output(ENA_PIN, GPIO.LOW)  # Disable motor
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
