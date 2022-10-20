import math
import serial
import time
# import numpy as np
# import RPi.GPIO as GPIO

# from controller import HexapodController

class SerialOperator(object):
    # def __init__(self, port, baudrate, timeout = 0.1):
    #     pass

    # def write_serial(self, instruction):
    #     pass

    # def read_serial(self):
    #     pass

    def __init__(self, port, baudrate, timeout = 0.1):
        self._serial = serial.Serial(port, baudrate, timeout = timeout)

        if self._serial.is_open == True:
            print('Open serial "{}" success, baudrate {}'.format(port, baudrate))
        else:
            print('Open serial "{}" failed'.format(baudrate))

    def write_serial(self, instruction):
        print('Write serial instruction : "{}"'.format(instruction))
        self._serial.write(instruction.encode('utf-8'))

    def read_serial(self):
        str = self._serial.readall().decode('utf-8')
        return str

class ServoController(object):
    def __init__(self, pulse_min = 544, pulse_max = 2400, freq = 50):
        # self.__serial = SerialOperator('/dev/tty.usbmodem56EC5F4536371', 9600, 0.1)
        self.__serial = SerialOperator('/dev/ttyACM0', 9600, 0.1)
        self.__cmd = ''

        self.__pulse_min = pulse_min
        self.__pulse_max = pulse_max

    def angle2pulse(self, km_angle):
        # return (self.__pulse_min + self.__pulse_max) / 2 + km_angle * ((self.__pulse_max - self.__pulse_min) / 180)
        i = (2500 - 500) / 180;
        return km_angle * i + 500;

    def set_angle(self, channel: int, angle: float):
        pulse = int(self.angle2pulse(angle))
        self.__cmd = self.__cmd + '#{}P{}'.format(channel, pulse)

    def run(self):
        if '' == self.__cmd:
            return

        self.__cmd = self.__cmd + 'T0D0\r\n'
        self.__serial.write_serial(self.__cmd)

        # while True:
        #     ack = self.__serial.read_serial()
        #     if 'OK' == ack:
        #         print('Write CMD success')
        #         break

        self.__cmd = ''


def main():
    servo_controller = ServoController()
    servo_channel = input('Please set servo channel :')

    while(True):
        angle = input('Please set angle :')
        servo_controller.set_angle(int(servo_channel), int(angle))
        servo_controller.run()


if __name__ == '__main__':
    main()