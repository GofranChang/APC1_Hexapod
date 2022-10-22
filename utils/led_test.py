import RPi.GPIO as GPIO
import time
import math
import threading

def init(gpiox):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpiox,GPIO.OUT)

def gpio_high(gpiox):
    GPIO.output(gpiox, GPIO.HIGH)

def gpio_low(gpiox):
    GPIO.output(gpiox, GPIO.LOW)

def clean():
    GPIO.cleanup()

gLedMode = 0

def led():
    while True:
        if 0 == gLedMode:
            gpio_low(12)

        if 1 == gLedMode:
            gpio_high(12)

        if 2 == gLedMode:
            gpio_high(12)
            time.sleep(1)
            gpio_low(12)
            time.sleep(1)

        if 3 == gLedMode:
            gpio_high(12)
            time.sleep(0.5)
            gpio_low(12)
            time.sleep(0.5)
            gpio_high(12)
            time.sleep(0.5)
            gpio_low(12)
            time.sleep(0.5)
            gpio_high(12)
            time.sleep(2)


def main():
    init(12)

    t = threading.Thread(target = led)
    t.start()

    while(True):
        global gLedMode
        gLedMode = input('Please set led mode :')

if __name__ == '__main__':
    main()