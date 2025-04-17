import time
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite
import threading

# GPIO Pin Setup
TRIG = 0 # Ultrasonic Trigger Pin                (Physical = 27)
ECHO = 1 # Ultrasonic Echo Pin                   (Physical = 28)
STEP_PIN = 17 # Top motor step pin               (Physical = 11)
DIR_PIN = 27 # Top motor direction pin           (Physical = 13)
ENA_PIN = 22 # Top motor enable pin              (Physical = 15)

STEP_PIN_2 = 6 # Lower left motor step pin       (Physical = 31)
DIR_PIN_2 = 12 # Lower left motor direction pin  (Physical = 32)
ENA_PIN_2 = 13 # Lower left motor enable pin     (Physical = 33)

STEP_PIN_3 = 23 # Lower right motor step pin     (Physical = 16)
DIR_PIN_3 = 25 # Lower right motor direction pin (Physical = 22)
ENA_PIN_3 = 5 # Lower right motor enable pin     (Physical = 29)

# Physical GPIO: 3 5 7 8  10 11 12 13 15 16 18 19 21 22 23 24 26 27 28 29 31 32 33 35 36 37 38 40 (All Physical)
# Digital  GPIO: 2 3 4 14 15 17 18 27 22 23 24 10 9  25 11 8  7  0  1  5  6  12 13 19 16 26 16 21 (All Digital)

# Physical GPIO: 11 13 15 31 32 33 16 22 29 3  5  7  8  10 12 18 19 21 23 24 26 27 28 35 (Used)
# Physical GPIO: 36 37 38 40 (Free)
# Digital  GPIO: 16 26 16 21 (Free)

# 7-Segment display pins (A to G, DP)
# Physical Pins: A = 3 B = 5 C = 7 D = 8
# Physical Pins: E = 10 F = 12 G = 18 DP = 19
SEGMENT_PINS = {
    'A': 2, 'B': 3, 'C': 4, 'D': 14,
    'E': 15, 'F': 18, 'G': 24, 'DP': 10
}

# Digit select pins
DIGIT_PINS = [9, 11, 8, 7] # Physical Pins: 21 23 24 26

# Motor and Ultrasonic GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG, GPIO.OUT) # Ultrasonic sensor
GPIO.setup(ECHO, GPIO.IN)

GPIO.setup(STEP_PIN, GPIO.OUT) # Top motor
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.output(ENA_PIN, GPIO.HIGH) # Enable first stepper motor (HIGH = On)

GPIO.setup(STEP_PIN_2, GPIO.OUT) # Lower left motor
GPIO.setup(DIR_PIN_2, GPIO.OUT)
GPIO.setup(ENA_PIN_2, GPIO.OUT)
GPIO.output(ENA_PIN_2, GPIO.HIGH) # Enable second motor

GPIO.setup(STEP_PIN_3, GPIO.OUT) # Lower right motor
GPIO.setup(DIR_PIN_3, GPIO.OUT)
GPIO.setup(ENA_PIN_3, GPIO.OUT)
GPIO.output(ENA_PIN_3, GPIO.HIGH) # Enable third motor

LED_PIN = 19 # (Physical = 35)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)  # Start with LED off

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
total_value = 0.0

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
    global total_value, current_display, display_timeout
    while True:
        if time.time() >= display_timeout:
            current_display = "aaaa" # Timeout reached, stop displaying
            total_value = 0.0 # Timeout reached, reset payout value
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
        time.sleep(0.001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.001)

# Rotates the lower left stepper motor by 180 degrees in determined direction for sorting
def rotate_stepper_motor_2(steps, direction):
    GPIO.output(DIR_PIN_2, direction) # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN_2, GPIO.HIGH)
        time.sleep(0.001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN_2, GPIO.LOW)
        time.sleep(0.001)

# Rotates the lower right stepper motor by 180 degrees in determined direction for sorting
def rotate_stepper_motor_3(steps, direction):
    GPIO.output(DIR_PIN_3, direction) # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN_3, GPIO.HIGH)
        time.sleep(0.001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN_3, GPIO.LOW)
        time.sleep(0.001)

# Caclulate value to display and handle input format
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
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn ON LED
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            # Perform action based on classification and rotate the first motor accordingly
            if prediction == 0:
                payout_value = plastic_value
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # Plastic -> push left = 180 clockwise (top motor)
                rotate_stepper_motor_2(1600, GPIO.LOW) # Plastic -> push right = 180 counterclockwise (lower left motor)
            elif prediction == 1:
                payout_value = aluminum_value
                print("Sorting: Aluminum")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Aluminum -> push right = 180 counterclockwise (top motor)
                rotate_stepper_motor_3(1600, GPIO.HIGH) # Aluminum -> push left = 180 clockwise (lower right motor)
            elif prediction == 2:
                payout_value = glass_value
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Glass -> push right = 180 counterclockwise (top motor)
                rotate_stepper_motor_3(1600, GPIO.LOW) # Glass -> push right = 180 counterclockwise (lower right motor)
            elif prediction == 3:
                payout_value = none_value
                print("Unknown Item")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # None -> push left = 180 clockwise (top motor)
                rotate_stepper_motor_2(1600, GPIO.HIGH) # None -> push left = 180 clockwise (lower left motor)

            calculateChange(payout_value)

            time.sleep(3) # Wait 3 seconds before the next detection
            GPIO.output(LED_PIN, GPIO.LOW) # Turn off LED, next item able to be inserted
            
        time.sleep(1) # Wait 1 second between checks so ultrasonic sensor doesn't spam

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.output(ENA_PIN, GPIO.LOW) # Disable top motor
    GPIO.output(ENA_PIN_2, GPIO.LOW) # Disable lower left motor
    GPIO.output(ENA_PIN_3, GPIO.LOW) # Disable lower right motor
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
