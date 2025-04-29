import RPi.GPIO as GPIO
import time

# GPIO setup
ARDUINO_SIGNAL_PIN = 18  # Use the pin number you connected to (GPIO 18 = pin 12)
GPIO.setmode(GPIO.BCM)
GPIO.setup(ARDUINO_SIGNAL_PIN, GPIO.IN)

try:
    print("Waiting for object detection signal from Arduino...")

    while True:
        if GPIO.input(ARDUINO_SIGNAL_PIN) == GPIO.HIGH:
            print("✅ Object detected within 13 cm!")
        else:
            print("❌ No object or too far.")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Exiting gracefully...")
finally:
    GPIO.cleanup()
