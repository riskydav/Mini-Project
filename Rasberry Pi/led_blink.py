import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

# PIN LED
led = 11
GPIO.setup(led, GPIO.OUT)

while True:
    GPIO.output(led, 1)
    print("Led ON")
    time.sleep(1)
    GPIO.output(led, 0)
    print("Led OFF")
    time.sleep(1)
