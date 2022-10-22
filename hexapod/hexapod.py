import math
import serial
import time
import numpy as np
import RPi.GPIO as GPIO

from controller import HexapodController

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

    def run(self, speed = 0):
        if '' == self.__cmd:
            return

        self.__cmd = self.__cmd + 'T' + str(speed) + 'D0\r\n'
        self.__serial.write_serial(self.__cmd)

        # while True:
        #     ack = self.__serial.read_serial()
        #     if 'OK' == ack:
        #         print('Write CMD success')
        #         break

        self.__cmd = ''

def constrain(x: float, a: float, b: float) -> float:
    assert a <= b
    if x < a:
        return a
    elif x > b:
        return b
    else:
        return x

class Hexapod(object):
    #                ^
    #       leg6     |     leg1
    #         \   forward   /
    #          \           /
    #           \---------/
    #           |         |
    #           |         |
    # leg5 -----|         |----- leg2
    #           |         |
    #           |         |
    #           /---------\
    #          /           \
    #         /             \
    #       leg4           leg3

    LEG1_IDX = 0
    LEG2_IDX = 1
    LEG3_IDX = 2
    LEG4_IDX = 3
    LEG5_IDX = 4
    LEG6_IDX = 5

    HEAD_X_IDX = 0
    HEAD_Y_IDX = 1

    COXA_IDX  = 0
    FEMUR_IDX = 1
    TIBIA_IDX = 2

    RAD_TO_DEG = 57.29577951

    TRIPOD_CASE = [1, 2, 1, 2, 1, 2]

    HOME_X = [ 63.0,  0.0,  -63.0, -63.0,  0.0,   63.0]  #coxa-to-toe home positions
    HOME_Y = [ 63.0,  89.0,  63.0, -63.0, -89.0, -63.0]
    HOME_Z = [-70.0, -70.0, -70.0, -70.0, -70.0, -70.0]

    COXA_CAL  = [0, 0, -5, 0, 0, 0]
    FEMUR_CAL = [0, 0, 0, 0, 0, 0]
    TIBIA_CAL = [-24, -22, -11, -6, -15, -25]

    HOME_HEAD_X = 90
    HOME_HEAD_Y = 90

    HEAD_X_CAL = -5
    HEAD_Y_CAL = -10

    M_PI = 3.141592

    def __init__(self):
        # leg part lengths
        self.COXA_LENGTH  = 46
        self.FEMUR_LENGTH = 43
        self.TIBIA_LENGTH = 91

        self.LEG_NAMES = ['right_front', 'right_middle', 'right_rear',
                          'left_rear', 'left_middle', 'left_front']

        self.__servo_controller = ServoController()
        # hexpad has 6 legs, every leg has 3 servos
        self.servo = np.zeros((6, 3), dtype = int)

        self.head_x_servo = 0
        self.head_y_servo = 0

        self.gait_speed = 0
        self.FRAME_TIME_MS = 20

        self.BODY_X = [ 110.9,  0.0, -109.9, -109.9,    0.0, 110.9]  #body center-to-coxa servo distances
        self.BODY_Y = [  59.1, 72.2,   59.1,  -59.1,  -72.2, -59.1]
        self.BODY_Z = [   0.0,  0.0,    0.0,    0.0,    0.0,   0.0]

        self.offset_X: List[float] = [0] * 6
        self.offset_Y: List[float] = [0] * 6
        self.offset_Z: List[float] = [0] * 6
        self.current_X: List[float] = [0] * 6
        self.current_Y: List[float] = [0] * 6
        self.current_Z: List[float] = [0] * 6

        self.tick = 0

        self.__controller = HexapodController('TcpRemoter')

        self.__current_time = Hexapod.get_current_time_ms()
        self.__previous_time = 0

    @staticmethod
    def get_current_time_ms():
        return round(time.time() * 1000)

    def loop(self):
        #set up frame time
        while True:
            self.__current_time = self.get_current_time_ms()
            if self.__current_time - self.__previous_time < self.FRAME_TIME_MS:
                continue

            cmd, cmd_x, cmd_y, cmd_r, head_x, head_y = self.__controller.get_command()

            if '' != cmd:
                pass
                # print('----------------------')
                # print(cmd)
                # print(cmd_x)
                # print(cmd_y)
                # print(cmd_r)

            if 'standby' == cmd:
                continue

            # Reset all
            if 'reset' == cmd:
                self.reset_head()
                self.reset_all()

            if 'move' == cmd:
                self.tripod_gait(cmd_x, cmd_y, cmd_r)
                self.__servo_controller.run()

            if 'head' == cmd:
                print('Head {} {}'.format(head_x, head_y))
                self.set_head(head_x, head_y)

            self.__previous_time = self.__current_time
            time.sleep(self.FRAME_TIME_MS / 1000)

    def set_led_fan_gpio(self, pin):
        self.led_fan_gpio = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin,GPIO.OUT)
        print(pin)
        GPIO.output(pin, True)

    def set_leg_servo_channel(self, leg_index, index, channel):
        self.servo[leg_index][index] = channel

    def set_head_servo_channel(self, idx, channel):
        if idx == Hexapod.HEAD_X_IDX:
            self.head_x_servo = channel

        if idx == Hexapod.HEAD_Y_IDX:
            self.head_y_servo = channel

    def leg_IK(self, leg_idx: int, x: float, y: float, z: float):
        l0 = math.sqrt(pow(x, 2) + pow(y, 2)) - self.COXA_LENGTH
        l3 = math.sqrt(pow(l0,2) + pow(z, 2))

        if l3 >= self.TIBIA_LENGTH + self.FEMUR_LENGTH or l3 <= self.TIBIA_LENGTH - self.FEMUR_LENGTH:
            return

        # compute tibia angle
        phi_tibia = math.acos((pow(self.FEMUR_LENGTH, 2) + pow(self.TIBIA_LENGTH, 2) - pow(l3, 2)) / (2 * self.FEMUR_LENGTH * self.TIBIA_LENGTH))
        theta_tibia = phi_tibia * Hexapod.RAD_TO_DEG - 23.0 + self.TIBIA_CAL[leg_idx]
        theta_tibia = constrain(theta_tibia, 0.0, 180.0)

        # compute femur angle
        gamma_femur = math.atan2(z, l0)
        # phi_femur = math.acos((sq(FEMUR_LENGTH) + sq(l3) - sq(TIBIA_LENGTH))/(2 * FEMUR_LENGTH * l3))
        phi_femur = math.acos((pow(self.FEMUR_LENGTH, 2) + pow(l3, 2) - pow(self.TIBIA_LENGTH, 2)) / (2 * self.FEMUR_LENGTH * l3))
        theta_femur = (phi_femur + gamma_femur) * Hexapod.RAD_TO_DEG + 14.0 + 90.0 + self.FEMUR_CAL[leg_idx]
        theta_femur = constrain(theta_femur, 0.0, 180.0)

        # compute coxa angle
        theta_coxa = math.atan2(x, y) * Hexapod.RAD_TO_DEG + self.COXA_CAL[leg_idx]

        if Hexapod.LEG1_IDX == leg_idx:
            theta_coxa = theta_coxa + 45.0

        elif Hexapod.LEG2_IDX == leg_idx:
            theta_coxa = theta_coxa + 90.0

        elif Hexapod.LEG3_IDX == leg_idx:
            theta_coxa = theta_coxa + 135.0

        elif Hexapod.LEG4_IDX == leg_idx:
            if theta_coxa < 0:
                theta_coxa = theta_coxa + 225.0
            else:
                theta_coxa = theta_coxa - 135.0

        elif Hexapod.LEG5_IDX == leg_idx:
            if theta_coxa < 0:
                theta_coxa = theta_coxa + 270.0
            else:
                theta_coxa = theta_coxa - 90.0

        elif Hexapod.LEG6_IDX == leg_idx:
            if theta_coxa < 0:
                theta_coxa = theta_coxa + 315.0
            else:
                theta_coxa = theta_coxa - 45.0

        theta_coxa = constrain(theta_coxa, 0.0, 180.0)
        # print('coxa : {}, femur : {}, tibia : {}'.format(theta_coxa, theta_femur, theta_tibia))

        if leg_idx <= Hexapod.LEG3_IDX:
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.COXA_IDX], theta_coxa)
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.FEMUR_IDX], theta_femur)
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.TIBIA_IDX], theta_tibia)
        else:
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.COXA_IDX], theta_coxa)
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.FEMUR_IDX], 180 - theta_femur)
            self.__servo_controller.set_angle(self.servo[leg_idx][Hexapod.TIBIA_IDX], 180 - theta_tibia)

    def compute_strides(self, commandedX, commandedY, commandedR):
        #compute stride lengths
        self.strideX = 90 * commandedX / 127
        self.strideY = 90 * commandedY / 127
        self.strideR = 35 * commandedR / 127

        #compute rotation trig
        self.sinRotZ = math.sin(math.radians(self.strideR))
        self.cosRotZ = math.cos(math.radians(self.strideR))

        #set duration for normal and slow speed modes
        if self.gait_speed == 0:
            self.duration = 480
        else:
            self.duration = 600

    def compute_amplitudes(self, leg_num: int):
        #compute total distance from center of body to toe
        self.totalX = self.HOME_X[leg_num] + self.BODY_X[leg_num]
        self.totalY = self.HOME_Y[leg_num] + self.BODY_Y[leg_num]

        #compute rotational offset
        self.rotOffsetX = self.totalY*self.sinRotZ + self.totalX*self.cosRotZ - self.totalX
        self.rotOffsetY = self.totalY*self.cosRotZ - self.totalX*self.sinRotZ - self.totalY

        #compute X and Y amplitude and constrain to prevent legs from crashing into each other
        self.amplitudeX = ((self.strideX + self.rotOffsetX)/2.0)
        self.amplitudeY = ((self.strideY + self.rotOffsetY)/2.0)
        self.amplitudeX = constrain(self.amplitudeX,-50,50)
        self.amplitudeY = constrain(self.amplitudeY,-50,50)

        #compute Z amplitude
        if abs(self.strideX + self.rotOffsetX) > abs(self.strideY + self.rotOffsetY):
            self.amplitudeZ = self.step_height_multiplier * (self.strideX + self.rotOffsetX) /4.0
        else:
            self.amplitudeZ = self.step_height_multiplier * (self.strideY + self.rotOffsetY) / 4.0

    def tripod_gait(self, command_x, command_y, command_r):
        #read commanded values from controller
        commandedX = command_x
        commandedY = command_y
        commandedR = command_r

        if abs(commandedX) <= 15 and abs(commandedR) <= 15 and abs(commandedY) <= 15 and self.tick <= 0:
            return

        self.compute_strides(commandedX, commandedY, commandedR)
        numTicks = round(self.duration / self.FRAME_TIME_MS / 2.0)

        for leg_idx in range(Hexapod.LEG1_IDX, Hexapod.LEG6_IDX + 1):
            self.step_height_multiplier = 1.1
            self.compute_amplitudes(leg_idx)

            if 1 == Hexapod.TRIPOD_CASE[leg_idx]:
                # print('11111')
                self.current_X[leg_idx] = Hexapod.HOME_X[leg_idx] - self.amplitudeX*math.cos(Hexapod.M_PI*self.tick/numTicks)
                self.current_Y[leg_idx] = Hexapod.HOME_Y[leg_idx] - self.amplitudeY*math.cos(Hexapod.M_PI*self.tick/numTicks)
                self.current_Z[leg_idx] = Hexapod.HOME_Z[leg_idx] + abs(self.amplitudeZ)*math.sin(Hexapod.M_PI*self.tick/numTicks)
                if self.tick >= numTicks-1:
                    Hexapod.TRIPOD_CASE[leg_idx] = 2

            elif 2 == Hexapod.TRIPOD_CASE[leg_idx]:
                # print('22222')
                self.current_X[leg_idx] = Hexapod.HOME_X[leg_idx] + self.amplitudeX*math.cos(Hexapod.M_PI*self.tick/numTicks)
                self.current_Y[leg_idx] = Hexapod.HOME_Y[leg_idx] + self.amplitudeY*math.cos(Hexapod.M_PI*self.tick/numTicks)
                self.current_Z[leg_idx] = Hexapod.HOME_Z[leg_idx]
                if self.tick >= numTicks-1:
                    Hexapod.TRIPOD_CASE[leg_idx] = 1

        if self.tick < numTicks - 1:
            self.tick += 1
        else:
            self.tick = 0

        # self.run()
        for leg_idx in range(Hexapod.LEG1_IDX, Hexapod.LEG6_IDX + 1):
            self.leg_IK(leg_idx,self.current_X[leg_idx]+self.offset_X[leg_idx],self.current_Y[leg_idx]+self.offset_Y[leg_idx],self.current_Z[leg_idx]+self.offset_Z[leg_idx])

    def reset_all(self):
        self.leg_IK(Hexapod.LEG1_IDX, 63, 63, -70)
        self.leg_IK(Hexapod.LEG2_IDX, 0, 89, -70)
        self.leg_IK(Hexapod.LEG3_IDX, -63, 63, -70)
        self.leg_IK(Hexapod.LEG4_IDX, -63, -63, -70)
        self.leg_IK(Hexapod.LEG5_IDX, 0, -89, -70)
        self.leg_IK(Hexapod.LEG6_IDX, 63, -63, -70)

        self.__servo_controller.run()

    def reset_head(self):
        # UP Down
        self.__servo_controller.set_angle(self.head_y_servo, Hexapod.HOME_HEAD_Y + Hexapod.HEAD_Y_CAL)
        # Left Right
        self.__servo_controller.set_angle(self.head_x_servo, Hexapod.HOME_HEAD_X + Hexapod.HEAD_X_CAL)
        self.__servo_controller.run()

    def map(self, x, inMin, inMax, outMin, outMax):
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

    def set_head(self, commandX, commandY):
        x_angel = self.map(commandX, -127, 127, 40, 140)
        y_angel = self.map(commandY, -127, 127, 70, 105)

        print('xxxxx {} {}', x_angel, y_angel)
        self.__servo_controller.set_angle(self.head_y_servo, y_angel)
        # Left Right
        self.__servo_controller.set_angle(self.head_x_servo, x_angel)
        self.__servo_controller.run(40)

    def run(self):
        self.__servo_controller.run()
