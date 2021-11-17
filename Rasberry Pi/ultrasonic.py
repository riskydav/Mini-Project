import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

# PIN
trig = 18
echo = 16

GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

while True:
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34000)/2
    dist = round(distance, 2)

    print("Distance", distance)
    time.sleep(1)
