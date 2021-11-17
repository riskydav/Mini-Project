import Jetson.GPIO as GPIO
import time

# PIN LED
led = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(led, GPIO.OUT, initial=GPIO.HIGH)

while True:
    GPIO.output(led, GPIO.HIGH)
    print("Led ON")
    time.sleep(2)
    GPIO.output(led, GPIO.LOW)
    print("Led OFF")
    time.sleep(2)
