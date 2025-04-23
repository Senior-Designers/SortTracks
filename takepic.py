import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2

# Ultrasonic pins (problematic ones)
TRIG = 17  # GPIO 0 (Physical pin 27)
ECHO = 20 # GPIO 1 (Physical pin 28)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

try:
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    print("[INFO] GPIO setup complete.")
except Exception as e:
    print(f"[ERROR] Failed to set up ultrasonic GPIOs: {e}")

# Distance function
def get_distance(timeout=1.0):
    GPIO.output(TRIG, False)
    time.sleep(0.05)  # Let sensor settle

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    timeout_time = start_time + timeout

    # Wait for ECHO to go HIGH (start of bounce)
    while GPIO.input(ECHO) == 0:
        if time.time() > timeout_time:
            raise TimeoutError("Timeout waiting for ECHO HIGH (object not detected or wiring issue).")
        pass
    pulse_start = time.time()

    # Wait for ECHO to go LOW (end of bounce)
    while GPIO.input(ECHO) == 1:
        if time.time() > timeout_time:
            raise TimeoutError("Timeout waiting for ECHO LOW (echo stuck HIGH).")
        pass
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2
    return distance


# Try camera
try:
    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration(main={"size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Give it time to warm up

    print("[INFO] Camera started successfully.")
    picam2.capture_file("test_image.jpg")
    print("[SUCCESS] Image captured and saved as 'test_image.jpg'.")

except Exception as e:
    print(f"[ERROR] Failed to start or use camera: {e}")

# Try ultrasonic sensor
try:
    print("[INFO] Measuring distance...")
    dist = get_distance()
    print(f"[SUCCESS] Measured distance: {dist:.2f} cm")
except Exception as e:
    print(f"[ERROR] Failed to read from ultrasonic sensor: {e}")

finally:
    GPIO.cleanup()
    print("[INFO] GPIO cleanup complete.")
