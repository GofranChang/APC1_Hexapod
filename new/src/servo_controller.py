import serial
import logger

class ServoController(object):
    """
    Example communication with PAN1322 BT module.
    Some commands do not respond with OK but with a '+...' line. This is
    implemented via command_with_event_response and handle_event, because
    '+...' lines are also used for real events.
    """

    def __init__(self, serialPort, baudrate = 115200):
        print('Init servo controller, serial port {}'.format(serialPort))

        serialPort = serialPort.replace(' ', '')
        self.__serialPort = serial.Serial(serialPort, baudrate, timeout = 0.1)
        if self.__serialPort.is_open == True:
            logger.g_logger.info('Open serial "{}" success, baudrate {}'.format(serialPort, baudrate))
        else:
            logger.g_logger.err('Open serial "{}" failed'.format(baudrate))

        self.__cmd = ''

    def __angle2pulse(self, angle):
        """
        Angle convert to pulse
        """
        return angle * ((2500 - 500) / 180) + 500

    def setAngle(self, channel, angle):
        """
        Set servo angle
        """
        pulse = int(self.__angle2pulse(angle))
        self.__cmd = self.__cmd + '#{}P{}'.format(channel, pulse)

    def exec(self, speed = 0, delay = 0):
        if '' == self.__cmd:
            return

        self.__cmd = '{}T{}D{}\r\n'.format(self.__cmd, speed, delay)

        logger.g_logger.trace('Write serial cmd : "{}"'.format(self.__cmd))
        self.__serialPort.write(self.__cmd.encode('utf-8'))

        self.__cmd = ''