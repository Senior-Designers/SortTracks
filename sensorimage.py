#!/usr/bin/env python3
import time
import threading
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# === CONFIGURE THIS TO MATCH YOUR DISPLAY TYPE ===
COMMON_ANODE = False   # ← set to False if your display is common‑cathode

def set_segment(pin, on):
    """Turn a segment on or off according to display type."""
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)

def set_digit(pin, on):
    """Enable or disable a digit's common pin."""
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)
    else:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)

# === your 7‑segment pin mapping ===
SEGMENT_PINS = {
    'A': 5,   # BCM5  (pin29)
    'B': 6,   # BCM6  (pin31)
    'C': 12,  # BCM12 (pin32)
    'D': 13,  # BCM13 (pin33)
    'E': 15,  # BCM16 (pin36)
    'F': 19,  # BCM19 (pin35)
    'G': 20,  # BCM20 (pin38)
    'DP': 26  # BCM26 (pin37)
}
DIGIT_PINS = [25, 18, 8, 4]  # left→right

# segments lit for each digit character
DIGIT_SEGMENTS = {
    '0': ['A','B','C','D','E','F'],
    '1': ['B','C'],
    '2': ['A','B','G','E','D'],
    '3': ['A','B','C','D','G'],
    '4': ['F','G','B','C'],
    '5': ['A','F','G','C','D'],
    '6': ['A','F','E','D','C','G'],
    '7': ['A','B','C'],
    '8': ['A','B','C','D','E','F','G'],
    '9': ['A','B','C','D','F','G'],
    ' ': [] 
}

# ultrasonic pins
TRIG_PIN = 23
ECHO_PIN = 24
THRESHOLD_CM = 10.0

# shared state for display
current_display = "    "    # 4 chars, blank = no segments
display_timeout = 0         # timestamp when to blank again

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # ultrasonic
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # 7‑segment segments
    for pin in SEGMENT_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        set_segment(pin, False)   # start with all segments off

    # digit commons
    for pin in DIGIT_PINS:
        GPIO.setup(pin, GPIO.OUT)
        set_digit(pin, False)     # start with all digits disabled

def get_distance_cm():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # wait for echo to go high
    while GPIO.input(ECHO_PIN) == 0:
        start = time.time()
    # wait for echo to go low
    while GPIO.input(ECHO_PIN) == 1:
        end = time.time()

    # speed of sound = 34300 cm/s, round‑trip so divide by 2
    return round((end - start) * 17150, 2)

def displayNumber(s: str):
    """Light each digit in turn (multiplex). s is a 4‑char string."""
    for i, ch in enumerate(s):
        # 1) turn off all digits
        for dp in DIGIT_PINS:
            set_digit(dp, False)

        # 2) set the segments for this character
        segs = DIGIT_SEGMENTS.get(ch, [])
        for seg, pin in SEGMENT_PINS.items():
            set_segment(pin, seg in segs)

        # 3) enable only this digit
        set_digit(DIGIT_PINS[i], True)

        time.sleep(0.002)  # dwell time

    # finally, turn all digits off again
    for dp in DIGIT_PINS:
        set_digit(dp, False)

def displayLoop():
    global current_display, display_timeout
    while True:
        # blank display if timeout has passed
        if time.time() >= display_timeout:
            current_display = "    "
        displayNumber(current_display)
        time.sleep(0.005)

def updateDisplay(val: int):
    """Show val (0–9999) for 5 seconds."""
    global current_display, display_timeout
    current_display = f"{val:04d}"
    display_timeout = time.time() + 5.0

def main():
    setup()
    # start the multiplexing thread
    threading.Thread(target=displayLoop, daemon=True).start()

    # initialize camera
    cam = Picamera2()
    cfg = cam.create_still_configuration()
    cam.configure(cfg)
    cam.start()

    counter = 0
    print("Starting...")

    try:
        while True:
            dist = get_distance_cm()
            if 0 < dist < THRESHOLD_CM:
                # take a picture
                timestamp = int(time.time())
                cam.capture_file("img.jpg")
                # increment counter by 5, wrap around at 10000
                counter = (counter + 1000) % 10000
                print(f"Captured image #{counter//5} → display {counter:04d}")
                updateDisplay(counter)
                time.sleep(1.0)   # debounce trigger

            time.sleep(0.05)      # poll interval
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("Cleaned up.")

if __name__ == "__main__":
    main()
