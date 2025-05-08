#!/usr/bin/env python3
import time
import threading
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite
from datetime import datetime
import os

# 9/10 aluminum
# 10/10 plastic
# 10/10 glass

# ————————————— PIN ASSIGNMENTS —————————————
# Ultrasonic Sensor GPIO
ARDUINO_SIGNAL_PIN = 23  # Physical: 16

# Stepper GPIOs
STEP_PIN   = 17; DIR_PIN   = 14; ENA_PIN   = 22 # Physical: 11, 8, 15 (top)          half works
STEP_PIN_2 = 27; DIR_PIN_2 = 7;  ENA_PIN_2 = 21 # Physical: 13, 26, 40 (lower left) doesn't work
STEP_PIN_3 = 9;  DIR_PIN_3 = 10; ENA_PIN_3 = 11 # Physical: 21, 19, 23 (lower right) works
# Physical(digital) 8(14) is dead, one to three of 13(27), 26(7), 40(21) is dead

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
interpreter = tflite.Interpreter(model_path="SortTracks/model.tflite")
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
def displayNumber(s:str, decimal_on):
    """One multiplex cycle for string s (length 4)."""
    for i,ch in enumerate(s):
        # 1) blank all digits
        for dp in DIGIT_PINS: set_digit(dp, False)

        # 2) set segments
        segs = DIGIT_SEGS.get(ch, [])

        if decimal_on and i == 1:  # After second number
            segs = segs + ['DP']  # turn ON decimal point

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
            displayNumber("    ", False)
        else:
            displayNumber(current_display, True)
        # next cycle in ~2ms

# Call this whenever you want to show a new value for 5s
def displayChange(val:int):
    global current_display, display_timeout
    current_display = f"{val:04d}"
    display_timeout = time.time() + 60.0

# Captures an image and converts it to UINT8 for the TFLite model
def capture_image():
    folder = "SortTracks/Testing" # Change to SortTracks/ClassName to save to that folder, then upload to Git
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    image_large = f"image_{timestamp}_large.jpg"

    picam2.capture_file(image_large) # Save image
    img = cv2.imread(image_large) # Read image
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) # Rotate image 90 degrees
    img = cv2.resize(img, (224, 224)) # Resize for model
    img = img.astype(np.uint8) # Convert to UINT8 (expected by model)
    img = np.expand_dims(img, axis=0) # Add batch dimension
    
    # Save the captured image to correct folder for further model training
    image_processed = os.path.join(folder, f"image_{timestamp}_processed.jpg")
    cv2.imwrite(image_processed, img[0])

    # Save current image, overwrite old one
    image_save = image_large
    image_save = "image_large.jpg"
    cv2.imwrite(image_save, img[0])
    os.remove(image_large) # Remove timestamped image from the cwd while keeping the timestamped one in the class folder safe

    print(f"Images saved:\n- Original: {image_save}\n- Processed: {image_processed}")
    
    return img

# Runs inference on an image and returns the predicted label
def classify_image(image):
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    confidence = np.max(output_data)
    predicted_class = np.argmax(output_data)

    log_message = f"Prediction values:\n{output_data}"
    print(log_message)

    #with open("SortTracks/log.txt", "a") as f:
        #f.write(log_message)
    
    if confidence < 40:
        predicted_class = 3 # Confidence too low (below 15.6%), select None
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
GPIO.setup(ARDUINO_SIGNAL_PIN, GPIO.IN)

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

interpreter = tflite.Interpreter(model_path="SortTracks/model2.tflite")
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]['index']
out = interpreter.get_output_details()[0]['index']

# ————————————— MAIN SORTING LOOP —————————————
try:
    while True:
        ARDUINO_SIGNAL_PIN = 23  # Physical: 16
        print("Waiting for object detection signal from Arduino...")

        if GPIO.input(ARDUINO_SIGNAL_PIN) == GPIO.HIGH:
            print("Object detected! Capturing image...")
            GPIO.output(LED_PIN, GPIO.HIGH) # Turn on LED
            time.sleep(2) # Wait for object to settle for better picture

            # Teachable Machine Inference
            image = capture_image()
            prediction = classify_image(image)

            # Turn motors and display change based on classification
            if prediction == 0: # Plastic
                payout_value = (payout_value + plastic_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # Plastic -> push left = 180 counterclockwise (top motor)
                time.sleep(0.5)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Plastic -> push right = 180 clockwise (both motor)

                rotate_stepper_motor_1(1600, GPIO.HIGH) # Reset 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Reset 180 clockwise (lower left motor)
            elif prediction == 1: # Aluminum
                payout_value = (payout_value + aluminum_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Aluminum")

                rotate_stepper_motor_1(1600, GPIO.LOW) # Aluminum -> push right = 180 clockwise (top motor)
                time.sleep(0.5)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Aluminum -> push right = 180 clockwise (lower right motor)

                rotate_stepper_motor_1(1600, GPIO.LOW) # Reset 180 clockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.LOW) # Reset 180 clockwise (lower right motor)
            elif prediction == 2: # Glass
                payout_value = (payout_value + glass_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, GPIO.LOW) # Glass -> push right = 180 clockwise (top motor)
                time.sleep(0.5)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Glass -> push left = 180 counterclockwise (lower right motor)

                rotate_stepper_motor_1(1600, GPIO.LOW) # Reset 180 clockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Reset 180 counterclockwise (lower right motor)
            elif prediction == 3: # None
                payout_value = (payout_value + none_value) % 10000 # Add return value and display on 7-Segment
                displayChange(payout_value)
                print("Unknown Item")
                rotate_stepper_motor_1(1600, GPIO.HIGH) # None -> push left = 180 counterclockwise (top motor)
                time.sleep(0.5)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # None -> push left = 180 counterclockwise (lower left motor)

                rotate_stepper_motor_1(1600, GPIO.HIGH) # Reset 180 counterclockwise (top motor)
                rotate_stepper_motors_2_3(1600, GPIO.HIGH) # Reset 180 counterclockwise (lower left motor)

            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(1)
        else:
            print("No signal yet")
        time.sleep(0.05)

except KeyboardInterrupt: # Shutdown
    GPIO.output(ENA_PIN,   GPIO.LOW)
    GPIO.output(ENA_PIN_2, GPIO.LOW)
    GPIO.output(ENA_PIN_3, GPIO.LOW)
    clear_display()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()