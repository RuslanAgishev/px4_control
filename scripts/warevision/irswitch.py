import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(15, GPIO.OUT)

GPIO.output(15, 0)
time.sleep(2)
GPIO.output(15, 1)
time.sleep(2)
GPIO.output(15, 0)