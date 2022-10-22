import serial

class SerialPortOperator(object):
    '''
    串口操作类
    '''

    def __init__(self, port, baudrate, timeout = 0.1):
        self.__serial = serial.Serial(port, baudrate, timeout = timeout)

        if self.__serial.is_open == True:
            print('Open serial "{}" success, baudrate {}'.format(port, baudrate))
        else:
            print('Open serial "{}" failed'.format(baudrate))