import RPi.GPIO as GPIO
import time
import threading

# Segment pins (A to G, DP)
# Random pin numbers, change based on wiring
SEGMENT_PINS = {
    'A': 5, 'B': 6, 'C': 13, 'D': 19,
    'E': 26, 'F': 12, 'G': 16, 'DP': 20
}

# Digit select pins
# Random pin numbers, change based on wiring
DIGIT_PINS = [21, 22, 23, 24]

# Set up pins to high/low
for pin in SEGMENT_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

for pin in DIGIT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Segment pins combinations corresponding to a number
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

# Display the passed in number on 7 segment display
def displayNumber(number_str):
    for i in range(4):
        GPIO.output(DIGIT_PINS[i], GPIO.HIGH)  # Activate digit
        num = number_str[i] # Number to display at current position
        segments = DIGIT_SEGMENTS.get(num, [])
      
        for seg, pin in SEGMENT_PINS.items():
          if seg == 'DP' and segments != []:
            GPIO.output(pin, GPIO.HIGH if i == 1 else GPIO.LOW) # Display decimal point after 2nd number
          else:
            GPIO.output(pin, GPIO.HIGH if seg in segments else GPIO.LOW) # Activate corresponding segments of the digit
        time.sleep(0.005)  # Small delay so you can see it
        GPIO.output(DIGIT_PINS[i], GPIO.LOW)  # Deactivate digit

# Changes displayed number, times out after 1 minute of no updates as low power functionality
def displayChange(new_display):
  global current_display, display_timeout
  current_display = new_display # Update current displayed number
  display_timeout = time.time() + 60 # Set timeout as 1 minute

# Check if timeout has been reached, if not then keep displaying
def updateDisplay():
  global current_display, display_timeout
  while True:
    if time.time() >= display_timeout:
      current_display = "aaaa" # Timeout reached, stop displaying
    displayNumber(current_display)
    time.sleep(0.01) # Delay for checks
