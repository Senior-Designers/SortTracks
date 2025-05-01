import RPi.GPIO as GPIO
import time

# Define GPIO Pins
TRIG = 23  # Trigger Pin (GPIO 23 / Pin 16)
ECHO = 24  # Echo Pin (GPIO 24 / Pin 18)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    # Ensure Trigger is LOW
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.1)

    # Send 10µs Trigger Pulse
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIG, GPIO.LOW)

    print("HEre")
    # Wait for Echo to go HIGH (start of pulse)
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        print("JGAHCVGJHSFC")
        start_time = time.time()

    print("HerE")
    # Wait for Echo to go LOW (end of pulse)
    end_time = time.time()
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # Calculate distance (speed of sound = 34300 cm/s)
    duration = end_time - start_time
    distance = (duration * 34300) / 2

    print("Here")
    return round(distance, 2)

try:
    while True:
        print("here")
        distance = measure_distance()
        print("he")
        print(f"Distance: {distance} cm")
        time.sleep(1)  # Delay before next reading

except KeyboardInterrupt:
    print("\nStopping program.")
    GPIO.cleanup()
