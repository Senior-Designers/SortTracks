import time
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite
import threading

# GPIO Pin Setup
TRIG = 23 # Ultrasonic Trigger Pin
ECHO = 24 # Ultrasonic Echo Pin
STEP_PIN = 17
DIR_PIN = 27
ENA_PIN = 22

# 7-Segment display pins (A to G, DP)
# Random pin numbers, change based on wiring
SEGMENT_PINS = {
    'A': 5, 'B': 6, 'C': 13, 'D': 19,
    'E': 26, 'F': 12, 'G': 16, 'DP': 20
}

# Digit select pins
# Random pin numbers, change based on wiring
DIGIT_PINS = [18, 25, 8, 7]

# Motor and Ultrasonic GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.output(ENA_PIN, GPIO.HIGH) # Enable the stepper motor (HIGH = On)

# 7-Segment Setup
for pin in SEGMENT_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
for pin in DIGIT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Load TFLite Model
interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

# Get input and output tensor indices
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Picamera2 setup
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (224, 224)})) # model-compatible size
picam2.start()

# Segment pins combinations corresponding to a number, a=off
DIGIT_SEGMENTS = {
    '0': ['A', 'B', 'C', 'D', 'E', 'F'],
    '1': ['B', 'C'],
    '2': ['A', 'B', 'G', 'E', 'D'],
    '3': ['A', 'B', 'C', 'D', 'G'],
    '4': ['F', 'G', 'B', 'C'],
    '5': ['A', 'F', 'G', 'C', 'D'],
    '6': ['A', 'F', 'E', 'D', 'C', 'G'],
    '7': ['A', 'B', 'C'],
    '8': ['A', 'B', 'C', 'D', 'E', 'F', 'G'],
    '9': ['A', 'B', 'C', 'D', 'F', 'G'],
    'a': []
}

current_display = "aaaa" # 4 a's -> nothing on
display_timeout = 0

# Display the passed in number on 7 segment display
def displayNumber(number_str):
    for i in range(4):
        GPIO.output(DIGIT_PINS[i], GPIO.HIGH) # Activate digit
        num = number_str[i] # Number to display at current position
        segments = DIGIT_SEGMENTS.get(num, [])
        
        for seg, pin in SEGMENT_PINS.items():
            if seg == 'DP' and segments != []:
                GPIO.output(pin, GPIO.HIGH if i == 1 else GPIO.LOW) # Display decimal point after 2nd number
            else:
                GPIO.output(pin, GPIO.HIGH if seg in segments else GPIO.LOW) # Activate corresponding segments of the digit
        time.sleep(0.005) # Small delay so you can see it
        GPIO.output(DIGIT_PINS[i], GPIO.LOW) # Deactivate current digit, only 1 can be on at a time, fast enough to look like all on at once

# Changes displayed number, times out after 1 minute of no updates as low power functionality
def displayChange(new_display):
    global current_display, display_timeout
    current_display = new_display # Update current displayed number
    display_timeout = time.time() + 60 # Set timeout as 1 minute

# Check if timeout has been reached, if not then keep displaying
def updateDisplay():
    global current_display, display_timeout
    while True:
        if time.time() >= display_timeout:
            current_display = "aaaa" # Timeout reached, stop displaying
        displayNumber(current_display)
        time.sleep(0.01) # Delay for checks

# Start 7-segment update loop in a background thread
threading.Thread(target=updateDisplay, daemon=True).start()

# Measures distance using an ultrasonic sensor
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001) # 10µs pulse
    GPIO.output(TRIG, False)

    start_time, stop_time = time.time(), time.time()
    
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2 # Convert to cm
    return distance
    
# Captures an image and converts it to UINT8 for the TFLite model
def capture_image():
    picam2.capture_file("image_large.jpg") # Save image
    img = cv2.imread("image_large.jpg") # Read image
    img = cv2.resize(img, (224, 224)) # Resize for model
    img = img.astype(np.uint8) # Convert to UINT8 (expected by model)
    img = np.expand_dims(img, axis=0) # Add batch dimension
    
    # Save the captured image without displaying it
    filename = "image_large.jpg"
    cv2.imwrite(filename, img[0])
    print(f"Image saved as {filename}")
    
    return img

# Runs inference on an image and returns the predicted label
def classify_image(image):
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    confidence = np.max(output_data)
    predicted_class = np.argmax(output_data)
    
    if confidence < 0.5:
        predicted_class = 3 # Confidence too low (below 50%), select none
    return predicted_class

# Rotates the first stepper motor by 180 degrees in determined direction for sorting
def rotate_stepper_motor_1(steps, direction):
    GPIO.output(DIR_PIN, direction) # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.0001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.0001)

total_value = 0.0

def calculateChange(payout_value):
    global total_value
    total_value += payout_value
    display_total_value = f"{int(total_value * 100):04d}" # Remove decimal and pad with 0's until string is 4 chars long
    displayChange(display_total_value) # Display the change value

# Main loop
try:
    while True:
        # Value in cents of payout
        payout_value = 0.0
        plastic_value = 0.1
        aluminum_value = 0.05
        glass_value = 0.03
        none_value = 0.0

        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 30.5: # Object detected within 1 foot, take picture
            print("Object detected! Capturing image...")
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            # Perform action based on classification and rotate the first motor accordingly
            if prediction == 0:
                payout_value = plastic_value
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # Plastic -> push left = 180 clockwise (top motor)
            elif prediction == 1:
                payout_value = aluminum_value
                print("Sorting: Aluminum")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Aluminum -> push right = 180 counterclockwise (top motor)
            elif prediction == 2:
                payout_value = glass_value
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Glass -> push right = 180 counterclockwise (top motor)
            elif prediction == 3:
                payout_value = none_value
                print("Unknown Item")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # None -> push left = 180 clockwise (top motor)

            calculateChange(payout_value)

        time.sleep(5) # Wait 5 seconds before the next detection

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.output(ENA_PIN, GPIO.LOW) # Disable motor
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
