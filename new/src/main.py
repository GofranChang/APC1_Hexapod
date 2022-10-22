# from hexapod import Hexapod
from servo_controller import ServoController
# from logger import Logger, g_logger, setLogger
import logger

def main():
    # TODO: Read configure file
    logger.setLogger('ACP1', logger.Logger.TRACE, True, 'logger.txt')
    logger.g_logger.info('Hexapod APC-1 system start')

if __name__ == '__main__':
    main()