#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# === CONFIGURE YOUR DISPLAY TYPE ===
COMMON_ANODE = False  # common‑cathode display

# === PIN MAPPINGS ===
SEGMENT_PINS = {
    'A': 5,   # BCM5
    'B': 6,   # BCM6
    'C': 12,  # BCM12
    'D': 13,  # BCM13
    'E': 15,  # BCM16
    'F': 19,  # BCM19
    'G': 20,  # BCM20
    'DP': 26  # BCM26
}
DIGIT_PINS = [25, 18, 8, 4]  # BCM pins for digits 1→4

# segments lit for each digit
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
}

def set_segment(pin, on):
    """Turn a segment on/off for common‑cathode or common‑anode."""
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)

def set_digit(pin, on):
    """Enable or disable one digit."""
    if COMMON_ANODE:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)
    else:
        GPIO.output(pin, GPIO.LOW  if on else GPIO.HIGH)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # initialize segment pins
    for pin in SEGMENT_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        set_segment(pin, False)
    # initialize digit pins
    for pin in DIGIT_PINS:
        GPIO.setup(pin, GPIO.OUT)
        set_digit(pin, False)

def clear_display():
    """Turn off all segments and digits."""
    for pin in SEGMENT_PINS.values():
        set_segment(pin, False)
    for pin in DIGIT_PINS:
        set_digit(pin, False)

def display_number(s: str):
    """One cycle of multiplexing the 4‑digit display for the string s."""
    for i, ch in enumerate(s):
        # turn all digits off
        for dp in DIGIT_PINS:
            set_digit(dp, False)
        # set segment pattern
        segments = DIGIT_SEGMENTS.get(ch, [])
        for seg, pin in SEGMENT_PINS.items():
            set_segment(pin, seg in segments)
        # enable this digit
        set_digit(DIGIT_PINS[i], True)
        time.sleep(0.002)  # hold long enough to be visible

def main():
    setup()
    clear_display()

    # show "0000" for 1 second before starting
    start = time.time()
    while time.time() - start < 1.0:
        display_number("0000")

    try:
        for val in range(0, 1001, 5):
            s = f"{val:04d}"          # zero-pad to 4 digits
            end = time.time() + 0.5   # display each value for 0.5s
            while time.time() < end:
                display_number(s)
    finally:
        clear_display()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
