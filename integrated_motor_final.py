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

total_value = 0.0

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

# Calculate change to display
def calculateChange(payout_value):
    total_value += payout_value
    display_total_value = str(total_value)
    # Remove the decimal point from display_total_value and if the number is smaller than 4 digits then add 0's to the left until it is 4 digits long
    

try:
    while True:
        payout_value = 0.0
        plastic_value = 0.1 # 10 cents
        aluminum_value = 0.05 # 5 cents
        glass_value = 0.03 # 3 cents
        none_value = 0.0 # 0 cents, why are you trying to recycle this item
        
        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 30.5:  # Object detected within 1 foot, take picture
            print("Object detected! Capturing image...")
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            # Perform action based on classification and rotate the first motor accordingly
            if prediction == 0:
                payout_value = plastic_value
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, direction=GPIO.HIGH) # Plastic -> push left = 180 clockwise (top motor)
            elif prediction == 1:
                payout_value = aluminum_value
                print("Sorting: Aluminum")
                rotate_stepper_motor_1(1600, direction=GPIO.LOW) # Aluminum -> push right = 180 counterclockwise (top motor)
            elif prediction == 2:
                payout_value = glass_value
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, direction=GPIO.LOW) # Glass -> push right = 180 counterclockwise (top motor)
            elif prediction == 3:
                payout_value = none_value
                print("Unknown Item")
                rotate_stepper_motor_1(1600, direction=GPIO.HIGH) # None -> push left = 180 clockwise (top motor)

        time.sleep(5)  # Wait 5 seconds before the next detection

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.output(ENA_PIN, GPIO.LOW)  # Disable motor
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
