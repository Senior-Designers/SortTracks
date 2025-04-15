import time
import RPi.GPIO as GPIO
import numpy as np
import cv2
import threading
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

# Stepper and Ultrasonic Pins
TRIG = 23
ECHO = 24
STEP_PIN = 17
DIR_PIN = 27
ENA_PIN = 22

# 7-Segment Display Pins
SEGMENT_PINS = {
    'A': 5, 'B': 6, 'C': 13, 'D': 19,
    'E': 26, 'F': 12, 'G': 16, 'DP': 20
}

DIGIT_PINS = [21, 22, 23, 24]

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor and Ultrasonic
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.output(ENA_PIN, GPIO.HIGH)  # Enable motor

# 7-Segment Setup
for pin in SEGMENT_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
for pin in DIGIT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

interpreter = tflite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (224, 224)}))
picam2.start()

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

current_display = "aaaa"
display_timeout = 0

def displayNumber(number_str):
    for i in range(4):
        GPIO.output(DIGIT_PINS[i], GPIO.HIGH)
        num = number_str[i]
        segments = DIGIT_SEGMENTS.get(num, [])
        for seg, pin in SEGMENT_PINS.items():
            if seg == 'DP' and segments:
                GPIO.output(pin, GPIO.HIGH if i == 1 else GPIO.LOW)
            else:
                GPIO.output(pin, GPIO.HIGH if seg in segments else GPIO.LOW)
        time.sleep(0.005)
        GPIO.output(DIGIT_PINS[i], GPIO.LOW)

def displayChange(new_display):
    global current_display, display_timeout
    current_display = new_display
    display_timeout = time.time() + 60

def updateDisplay():
    global current_display, display_timeout
    while True:
        if time.time() >= display_timeout:
            current_display = "aaaa"
        displayNumber(current_display)
        time.sleep(0.01)

threading.Thread(target=updateDisplay, daemon=True).start()

# ----------------------------- UTILS -----------------------------
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time, stop_time = time.time(), time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

def capture_image():
    picam2.capture_file("image_large.jpg")
    img = cv2.imread("image_large.jpg")
    img = cv2.resize(img, (224, 224))
    img = img.astype(np.uint8)
    img = np.expand_dims(img, axis=0)
    cv2.imwrite("image_large.jpg", img[0])
    return img

def classify_image(image):
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    confidence = np.max(output_data)
    predicted_class = np.argmax(output_data)
    if confidence < 0.5:
        predicted_class = 3
    return predicted_class

def rotate_stepper_motor_1(steps, direction):
    GPIO.output(DIR_PIN, direction)
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.0001)

total_value = 0.0

def calculateChange(payout_value):
    global total_value
    total_value += payout_value
    display_total_value = f"{int(total_value * 100):04d}"
    displayChange(display_total_value)

# Main loop
try:
    while True:
        payout_value = 0.0
        plastic_value = 0.1
        aluminum_value = 0.05
        glass_value = 0.03
        none_value = 0.0

        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 30.5:
            print("Object detected! Capturing image...")
            image = capture_image()
            prediction = classify_image(image)

            print(f"Predicted Class: {prediction}")

            if prediction == 0:
                payout_value = plastic_value
                print("Sorting: Plastic")
                rotate_stepper_motor_1(1600, GPIO.HIGH)
            elif prediction == 1:
                payout_value = aluminum_value
                print("Sorting: Aluminum")
                rotate_stepper_motor_1(1600, GPIO.LOW)
            elif prediction == 2:
                payout_value = glass_value
                print("Sorting: Glass")
                rotate_stepper_motor_1(1600, GPIO.LOW)
            elif prediction == 3:
                payout_value = none_value
                print("Unknown Item")
                rotate_stepper_motor_1(1600, GPIO.HIGH)

            calculateChange(payout_value)

        time.sleep(5)

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.output(ENA_PIN, GPIO.LOW)
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
