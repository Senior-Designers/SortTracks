#!/usr/bin/env python3
import time
import threading
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

# ————————————— PIN ASSIGNMENTS —————————————
# Ultrasonic Sensor GPIO
TRIG_PIN   = 23 # Physical: 16
ECHO_PIN   = 24 # Physical: 18

# Stepper GPIOs
STEP_PIN   = 17; DIR_PIN   = 14; ENA_PIN   = 22 # Physical: 11, 8, 15 (top)          half works
STEP_PIN_2 = 27; DIR_PIN_2 = 7;  ENA_PIN_2 = 21 # Physical: 13, 26, 40 (lower left) doesn't work
STEP_PIN_3 = 9;  DIR_PIN_3 = 10; ENA_PIN_3 = 11 # Physical: 21, 19, 23 (lower right) works
# Physical 8 is dead, one to three of 13, 26, 40 is dead

# LED
LED_PIN    = 19 # Physical: 35

# 7‑Segment GPIO (common‑cathode)
COMMON_ANODE  = False
SEGMENT_PINS  = {'A':5, 'B':6, 'C':12, 'D':13, 'E':15, 'F':19, 'G':20, 'DP':26}
DIGIT_PINS    = [25, 18, 8, 4] # left→right
DIGIT_SEGS    = {
  '0':['A','B','C','D','E','F'],'1':['B','C'],'2':['A','B','G','E','D'],
  '3':['A','B','C','D','G'],'4':['F','G','B','C'],'5':['A','F','G','C','D'],
  '6':['A','F','E','D','C','G'],'7':['A','B','C'],'8':['A','B','C','D','E','F','G'],
  '9':['A','B','C','D','F','G'],' ':[]
}

# ————————————— GLOBAL STATES —————————————
# Global variables for return values, 7-Segment display, and threaded timeout for 7-Segment
counter = 0
current_display = "    "
display_timeout = 0
payout_value = 0
plastic_value = 10
aluminum_value = 5
glass_value = 3
none_value = 0

# Load TFLite Model
interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

# Get input and output tensor indices
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# ————————————— GPIO HELPERS —————————————
# Set GPIO output states
def set_segment(pin, on):
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)

# Set GPIO output states
def set_digit(pin, on):
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)
    else:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)

# Clear the 7-Segment display
def clear_display():
    for p in SEGMENT_PINS.values(): set_segment(p, False)
    for p in DIGIT_PINS:            set_digit(p,   False)

# Flash each digit quickly to "stay on"
def displayNumber(s:str):
    """One multiplex cycle for string s (length 4)."""
    for i,ch in enumerate(s):
        # 1) blank all digits
        for dp in DIGIT_PINS: set_digit(dp, False)
        # 2) set segments
        segs = DIGIT_SEGS.get(ch, [])
        for seg,p in SEGMENT_PINS.items():
            set_segment(p, seg in segs)
        # 3) enable only digit i
        set_digit(DIGIT_PINS[i], True)
        time.sleep(0.002)

# ————————————— DISPLAY THREAD —————————————
# Keeps displaying the change back as long as the timeout has not been reached
def displayLoop():
    global current_display, display_timeout
    while True:
        if time.time() >= display_timeout:
            displayNumber("    ")
        else:
            displayNumber(current_display)
        # next cycle in ~2ms

# Call this whenever you want to show a new value for 5s
def displayChange(val:int):
    global current_display, display_timeout
    current_display = f"{val:04d}"
    display_timeout = time.time() + 10.0

# ————————————— DISTANCE —————————————
# Pings the ultrasonic sensor to check the distance of the nearest object (cone shaped)
def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    start,end = time.time(), time.time()
    while GPIO.input(ECHO_PIN)==0: start = time.time()
    while GPIO.input(ECHO_PIN)==1: end   = time.time()
    return (end-start)*17150

# Captures an image and converts it to UINT8 for the TFLite model
def capture_image():
    picam2.capture_file("image_large.jpg") # Save image
    img = cv2.imread("image_large.jpg") # Read image
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) # Rotate image 90 degrees
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

# ————————————— STEPPERS —————————————
# Rotates the top stepper motor by 180 degrees in determined direction for sorting
def rotate_stepper_motor_1(steps, direction):
    GPIO.output(DIR_PIN_3, direction) # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.001)

# Rotates the lower two stepper motors by 180 degrees in determined direction for sorting
def rotate_stepper_motors_2_3(steps, direction):
    GPIO.output(DIR_PIN_3, direction) # Set direction
    for _ in range(steps):
        GPIO.output(STEP_PIN_3, GPIO.HIGH)
        time.sleep(0.001) # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN_3, GPIO.LOW)
        time.sleep(0.001)

# ————————————— SETUP —————————————
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ultrasonic
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Steppers + LED GPIO setup
for p in (STEP_PIN,DIR_PIN,ENA_PIN, STEP_PIN_2,DIR_PIN_2,ENA_PIN_2, STEP_PIN_3,DIR_PIN_3,ENA_PIN_3, LED_PIN):
    GPIO.setup(p, GPIO.OUT)
GPIO.output(ENA_PIN,   GPIO.HIGH)
GPIO.output(ENA_PIN_2, GPIO.HIGH)
GPIO.output(ENA_PIN_3, GPIO.HIGH)
GPIO.output(LED_PIN,   GPIO.LOW)

# 7‑Segment
for p in SEGMENT_PINS.values(): GPIO.setup(p, GPIO.OUT)
for p in DIGIT_PINS:            GPIO.setup(p, GPIO.OUT)
clear_display()

# Start display thread
threading.Thread(target=displayLoop, daemon=True).start()

# Picamera2 + TFLite Setup
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size":(224,224)}))
picam2.start()

interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]['index']
out = interpreter.get_output_details()[0]['index']

# ————————————— MAIN SORTING LOOP —————————————
try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")

        if 0 < dist < 30.5: # Distance check for object
            print("Object detected! Capturing image...")
            GPIO.output(LED_PIN, GPIO.HIGH) # Turn on LED

            # Teachable Machine Inference
            image = capture_image()
            prediction = classify_image(image)

            # Turn motors and display change based on classification
            if prediction == 0: # Plastic
                payout_value = (payout_value + plastic_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # Plastic -> push left = 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Plastic -> push right = 180 clockwise (both motor)

                rotate_stepper_motor_1(1600, GPIO.HIGH) # Reset 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Reset 180 clockwise (lower left motor)
            elif prediction == 1: # Aluminum
                payout_value = (payout_value + aluminum_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Aluminum")

                rotate_stepper_motor_1(1600, GPIO.LOW) # Aluminum -> push right = 180 clockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Aluminum -> push left = 180 counterclockwise (lower right motor)

                rotate_stepper_motor_1(1600, GPIO.HIGH) # Reset 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Reset 180 clockwise (lower right motor)
            elif prediction == 2: # Glass
                payout_value = (payout_value + glass_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Glass -> push right = 180 clockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Glass -> push right = 180 clockwise (lower right motor)

                rotate_stepper_motor_1(1600, GPIO.LOW) # Reset 180 clockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Reset 180 clockwise (lower right motor)
            elif prediction == 3: # None
                payout_value = (payout_value + none_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Unknown Item")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # None -> push left = 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # None -> push left = 180 counterclockwise (lower left motor)

                rotate_stepper_motor_1(1600, GPIO.HIGH) # Reset 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Reset 180 counterclockwise (lower left motor)

            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(1)
        time.sleep(0.1)

except KeyboardInterrupt: # Shutdown
    GPIO.output(ENA_PIN,   GPIO.LOW)
    GPIO.output(ENA_PIN_2, GPIO.LOW)
    GPIO.output(ENA_PIN_3, GPIO.LOW)
    clear_display()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
