# from ..src import logger
import sys
sys.path.append(r'../src')
import logger

logger.setLogger('test', logger.Logger.TRACE, True, '.cache/logger.txt')
logger.g_logger.trace('Trace log test {}'.format(0))

# gLogger = logger.Logger('test', logger.Logger.TRACE, True, '.cache/logger.txt')
# gLogger.trace('Trace log test {}'.format(0))