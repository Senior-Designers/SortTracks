import RPi.GPIO as GPIO
import time

# GPIO Pin Setup
STEP_PIN = 17 # Top motor step pin
DIR_PIN = 27 # Top motor direction pin
ENA_PIN = 22 # Top motor enable pin

STEP_PIN_2 = 6 # Lower left motor step pin
DIR_PIN_2 = 12 # Lower left motor direction pin
ENA_PIN_2 = 13 # Lower left motor enable pin

STEP_PIN_3 = 23 # Lower right motor step pin
DIR_PIN_3 = 25 # Lower right motor direction pin
ENA_PIN_3 = 5 # Lower right motor enable pin

# Motor and Ultrasonic GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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

print("Stepper motor running... Press CTRL+C to stop.")

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

try:
    while True:
        rotate_stepper_motor_1(1600, GPIO.LOW)
        rotate_stepper_motor_2(1600, GPIO.HIGH)
        rotate_stepper_motor_3(1600, GPIO.LOW)
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopping motor and cleaning up GPIO...")
    GPIO.output(ENA_PIN, GPIO.LOW) # Disable top motor
    GPIO.output(ENA_PIN_2, GPIO.LOW) # Disable lower left motor
    GPIO.output(ENA_PIN_3, GPIO.LOW) # Disable lower right motor
    GPIO.cleanup()  # Reset GPIO pins
    print("GPIO cleanup done. Exiting program.")
