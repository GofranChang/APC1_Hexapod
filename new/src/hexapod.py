from ast import List
from servo_controller import ServoController
import numpy as np
import utils
import math
import logger
import time
import os

M_PI = 3.141592

RAD_TO_DEG = 57.29577951

def constrain(x: float, a: float, b: float) -> float:
    assert a <= b
    if x < a:
        return a
    elif x > b:
        return b
    else:
        return x

class APC1Cmd(object):
    def __init__(self):
        self.moveX = 0
        self.moveY = 0
        self.moveR = 0
        self.headX = 0
        self.headY = 0
        self.ledsAndFans = False
        pass


class APC1(object):
    """
                   ^
          leg6     |     leg1
            \   forward   /
             \           /
              \---------/
              |         |
              |         |
    leg5 -----|         |----- leg2
              |         |
              |         |
              /---------\
             /           \
            /             \
          leg4           leg3
    """

    def __init__(self):
        self.__coxaLength  = 46
        self.__femurLength = 43
        self.__tibiaLength = 91

        self.__leg1Idx = 0
        self.__leg2Idx = 1
        self.__leg3Idx = 2
        self.__leg4Idx = 3
        self.__leg5Idx = 4
        self.__leg6Idx = 5

        self.__coxaIdx  = 0
        self.__femurIdx = 1
        self.__tibiaIdx = 2

        self.__servoChannels = np.zeros((6, 3), dtype = int)

        # TODO: read from configure file
        self.__homeX = [ 63.0,  0.0,  -63.0, -63.0,  0.0,   63.0]
        self.__homeY = [ 63.0,  89.0,  63.0, -63.0, -89.0, -63.0]
        self.__homeZ = [-70.0, -70.0, -70.0, -70.0, -70.0, -70.0]

        self.__bodyX = [ 110.9,  0.0, -109.9, -109.9,    0.0, 110.9]
        self.__bodyY = [  59.1, 72.2,   59.1,  -59.1,  -72.2, -59.1]
        self.__bodyZ = [   0.0,  0.0,    0.0,    0.0,    0.0,   0.0]

        # TODO: read from configure file
        self.__coxaCal  = [0, 0, -5, 0, 0, 0]
        self.__femurCal = [0, 0, 0, 0, 0, 0]
        self.__tibiaCal = [-24, -22, -11, -6, -15, -25]

        self.__offsetX: List[float] = [0] * 6
        self.__offsetY: List[float] = [0] * 6
        self.__offsetZ: List[float] = [0] * 6
        self.__currentX: List[float] = [0] * 6
        self.__currentY: List[float] = [0] * 6
        self.__currentZ: List[float] = [0] * 6

        self.__homeHeadX = 90
        self.__homeHeadY = 90

        self.__headXCal = -5
        self.__headYCal = -10

        self.__headXAngle = self.__homeHeadX
        self.__headYAngle = self.__homeHeadY

        self.__tripodCase = [1, 2, 1, 2, 1, 2]

        self.__frameTimeMs = 20
        self.__tick = 0

    def initialize(self, servoCfgPath):
        logger.g_logger.info('Initialize hexapod, servo cfg file {}'.format(servoCfgPath))
        with open(servoCfgPath, 'r') as fr:
            for line in fr:
                cfg = line.split(':')
                if len(cfg) < 2:
                    continue
                logger.g_logger.trace('Servo channel {} : {}'.format(cfg[0], cfg[1]))

                if 'LEG' in cfg[0]:
                    legIdx = int(cfg[0].replace('LEG_', '')) - 1
                    if legIdx < self.__leg1Idx or legIdx > self.__leg6Idx:
                        logger.g_logger.warn('Unknown leg inx {}'.format(legIdx))
                        continue

                    channels = cfg[1].replace('\n', '').replace(' ', '').split(',')
                    self.__servoChannels[legIdx][self.__coxaIdx] = int(channels[0])
                    self.__servoChannels[legIdx][self.__femurIdx] = int(channels[1])
                    self.__servoChannels[legIdx][self.__tibiaIdx] = int(channels[2])

                elif 'HEAD' in cfg[0]:
                    headIdx = cfg[0].replace('HEAD_', '')
                    if 'X' in headIdx:
                        self.__headXServoChannel = int(cfg[1])
                    elif 'Y' in headIdx:
                        self.__headYServoChannel = int(cfg[1])

                elif 'SERIAL_PORT' in cfg[0]:
                    logger.g_logger.info('Servo controller serial port : "{}"'.format(cfg[1].replace('\n', '')))
                    self.__servoController = ServoController(cfg[1].replace('\n', ''))

        self.__currentTime = utils.getCurrentTimeMs()
        self.__previousTime = 0
        self.__gaitSpeedMode = 0

    def loop(self):
        while True:
            self.__currentTime = utils.getCurrentTimeMs()
            if self.__currentTime - self.__previousTime < self.__frameTimeMs:
                continue

            # self.__tripodGait(127, 0, 0)
            self.__headServo(-127, -127)
            self.__servoController.exec()

            self.__previousTime = self.__currentTime
            time.sleep(self.__frameTimeMs / 1000)

    def __headServo(self, x, y):
        if 0 != x:
            if x > 0:
                self.__headXAngle -= 1
            elif x < 0:
                self.__headXAngle += 1

            headXAngle = constrain(self.__headXAngle + self.__headXCal, 40, 140)
            logger.g_logger.warn('Head X : {}'.format(headXAngle))
            self.__servoController.setAngle(self.__headXServoChannel, headXAngle)

        if 0 != y:
            if y > 0:
                self.__headYAngle -= 1
            elif y < 0:
                self.__headYAngle += 1

            headYAngle = constrain(self.__headYAngle + self.__headYCal, 70, 105)
            logger.g_logger.warn('Head Y : {}'.format(headYAngle))
            self.__servoController.setAngle(self.__headYServoChannel, headYAngle)

    def __tripodGait(self, x, y, r):
        if abs(x) <= 15 and abs(r) <= 15 and abs(y) <= 15 and self.__tick <= 0:
            return

        self.__computeStrides(x, y, r)
        numTicks = round(self.__duration / self.__frameTimeMs / 2.0)

        for legIdx in range(self.__leg1Idx, self.__leg6Idx + 1):
            self.__stepHeightMultiplier = 1.1
            self.__computeAmplitudes(legIdx)

            if 1 == self.__tripodCase[legIdx]:
                self.__currentX[legIdx] = self.__homeX[legIdx] - self.__amplitudeX*math.cos(M_PI*self.__tick/numTicks)
                self.__currentY[legIdx] = self.__homeY[legIdx] - self.__amplitudeY*math.cos(M_PI*self.__tick/numTicks)
                self.__currentZ[legIdx] = self.__homeZ[legIdx] + abs(self.__amplitudeZ)*math.sin(M_PI*self.__tick/numTicks)
                if self.__tick >= numTicks-1:
                    self.__tripodCase[legIdx] = 2

            elif 2 == self.__tripodCase[legIdx]:
                self.__currentX[legIdx] = self.__homeX[legIdx] + self.__amplitudeX*math.cos(M_PI*self.__tick/numTicks)
                self.__currentY[legIdx] = self.__homeY[legIdx] + self.__amplitudeY*math.cos(M_PI*self.__tick/numTicks)
                self.__currentZ[legIdx] = self.__homeZ[legIdx]
                if self.__tick >= numTicks-1:
                    self.__tripodCase[legIdx] = 1

        if self.__tick < numTicks - 1:
            self.__tick += 1
        else:
            self.__tick = 0

        # self.run()
        for legIdx in range(self.__leg1Idx, self.__leg6Idx + 1):
            self.__legIk(legIdx, self.__currentX[legIdx] + self.__offsetX[legIdx], self.__currentY[legIdx] + self.__offsetY[legIdx], self.__currentZ[legIdx] + self.__offsetZ[legIdx])

    def __legIk(self, legIdx, x, y, z):
        l0 = math.sqrt(pow(x, 2) + pow(y, 2)) - self.__coxaLength
        l3 = math.sqrt(pow(l0,2) + pow(z, 2))

        if l3 >= self.__tibiaLength + self.__femurLength or l3 <= self.__tibiaLength - self.__femurLength:
            return

        # compute tibia angle
        phiTibia = math.acos((pow(self.__femurLength, 2) + pow(self.__tibiaLength, 2) - pow(l3, 2)) / (2 * self.__femurLength * self.__tibiaLength))
        thetaTibia = phiTibia * RAD_TO_DEG - 23.0 + self.__tibiaCal[legIdx]
        thetaTibia = constrain(thetaTibia, 0.0, 180.0)

        # compute femur angle
        gammaFemur = math.atan2(z, l0)
        phiFemur = math.acos((pow(self.__femurLength, 2) + pow(l3, 2) - pow(self.__tibiaLength, 2)) / (2 * self.__femurLength * l3))
        thetaFemur = (phiFemur + gammaFemur) * RAD_TO_DEG + 14.0 + 90.0 + self.__femurCal[legIdx]
        thetaFemur = constrain(thetaFemur, 0.0, 180.0)

        # compute coxa angle
        thetaCoxa = math.atan2(x, y) * RAD_TO_DEG + self.__coxaCal[legIdx]

        if self.__leg1Idx == legIdx:
            thetaCoxa = thetaCoxa + 45.0

        elif self.__leg2Idx == legIdx:
            thetaCoxa = thetaCoxa + 90.0

        elif self.__leg3Idx == legIdx:
            thetaCoxa = thetaCoxa + 135.0

        elif self.__leg4Idx == legIdx:
            if thetaCoxa < 0:
                thetaCoxa = thetaCoxa + 225.0
            else:
                thetaCoxa = thetaCoxa - 135.0

        elif self.__leg5Idx == legIdx:
            if thetaCoxa < 0:
                thetaCoxa = thetaCoxa + 270.0
            else:
                thetaCoxa = thetaCoxa - 90.0

        elif self.__leg6Idx == legIdx:
            if thetaCoxa < 0:
                thetaCoxa = thetaCoxa + 315.0
            else:
                thetaCoxa = thetaCoxa - 45.0

        thetaCoxa = constrain(thetaCoxa, 0.0, 180.0)
        logger.g_logger.trace('APC1 APC-1 leg ik : coxa : {}, femur : {}, tibia : {}'.format(thetaCoxa, thetaFemur, thetaTibia))

        if legIdx <= self.__leg3Idx:
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__coxaIdx], thetaCoxa)
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__femurIdx], thetaFemur)
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__tibiaIdx], thetaTibia)
        else:
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__coxaIdx], thetaCoxa)
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__femurIdx], 180 - thetaFemur)
            self.__servoController.setAngle(self.__servoChannels[legIdx][self.__tibiaIdx], 180 - thetaTibia)

    def __computeStrides(self, x, y, r):
        #compute stride lengths
        self.__strideX = 90 * x / 127
        self.__strideY = 90 * y / 127
        self.__strideR = 35 * r / 127

        #compute rotation trig
        self.__sinRotZ = math.sin(math.radians(self.__strideR))
        self.__cosRotZ = math.cos(math.radians(self.__strideR))

        #set __duration for normal and slow speed modes
        if self.__gaitSpeedMode == 0:
            self.__duration = 480
        else:
            self.__duration = 600

    def __computeAmplitudes(self, legIdx: int):
        #compute total distance from center of body to toe
        self.__totalX = self.__homeX[legIdx] + self.__bodyX[legIdx]
        self.__totalY = self.__homeY[legIdx] + self.__bodyY[legIdx]

        #compute rotational offset
        self.__rotOffsetX = self.__totalY * self.__sinRotZ + self.__totalX * self.__cosRotZ - self.__totalX
        self.__rotOffsetY = self.__totalY * self.__cosRotZ - self.__totalX * self.__sinRotZ - self.__totalY

        #compute X and Y amplitude and constrain to prevent legs from crashing into each other
        self.__amplitudeX = ((self.__strideX + self.__rotOffsetX) / 2.0)
        self.__amplitudeY = ((self.__strideY + self.__rotOffsetY) / 2.0)
        self.__amplitudeX = constrain(self.__amplitudeX, -50, 50)
        self.__amplitudeY = constrain(self.__amplitudeY, -50, 50)

        #compute Z amplitude
        if abs(self.__strideX + self.__rotOffsetX) > abs(self.__strideY + self.__rotOffsetY):
            self.__amplitudeZ = self.__stepHeightMultiplier * (self.__strideX + self.__rotOffsetX) / 4.0
        else:
            self.__amplitudeZ = self.__stepHeightMultiplier * (self.__strideY + self.__rotOffsetY) / 4.0


def main():
    logger.setLogger('test', logger.Logger.TRACE, True)

    apc1 = APC1()
    apc1.initialize(os.path.split(os.path.realpath(__file__))[0] + '/../resource/servo.ini')

    apc1.loop()


if __name__ == '__main__':
    main()