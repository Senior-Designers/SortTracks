import RPi.GPIO as GPIO
import time

STEP_PIN = 17  # PUL- (Pulled LOW)
DIR_PIN = 27   # DIR- (Pulled LOW)
ENA_PIN = 22   # ENA- (Pulled LOW)

# Setup GPIO Mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)

# Enable the motor driver (HIGH = On)
GPIO.output(ENA_PIN, GPIO.HIGH)  

# Set direction (Change to GPIO.LOW to reverse)
GPIO.output(DIR_PIN, GPIO.HIGH)  

print("Stepper motor running... Press CTRL+C to stop.")

try:
    while True:
        # Send step pulse
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.0001)  # Pulse width (Lower = Faster)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.0001) 

except KeyboardInterrupt:
    print("\nStopping motor and cleaning up GPIO...")
    GPIO.output(ENA_PIN, GPIO.LOW)  # Disable motor
    GPIO.cleanup()  # Reset GPIO pins
    print("GPIO cleanup done. Exiting program.")
