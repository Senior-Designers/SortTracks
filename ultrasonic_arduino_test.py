import RPi.GPIO as GPIO
import time

# GPIO setup
ARDUINO_SIGNAL_PIN = 23  # Physical: 16)
GPIO.setmode(GPIO.BCM)
GPIO.setup(ARDUINO_SIGNAL_PIN, GPIO.IN)

try:
    print("Waiting for object detection signal from Arduino...")

    while True:
        if GPIO.input(ARDUINO_SIGNAL_PIN) == GPIO.HIGH:
            print("Object detected within 13 cm")
        else:
            print("No signal yet")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Exiting gracefully...")
finally:
    GPIO.cleanup()