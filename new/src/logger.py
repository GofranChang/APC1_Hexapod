import spdlog

class Logger(object):
    TRACE    = 0
    DEBUG    = 1
    INFO     = 2
    WARN     = 3
    ERR      = 4
    CRITICAL = 5
    OFF      = 6

    def __init__(self, name, level = WARN, isConsole = False, logPath = ''):
        # pass
        # if not False and '' == logPath:
            # return

        self.__name = name
        self.__sinks = []

        if isConsole:
            self.__sinks.append(spdlog.stdout_color_sink_mt())
        
        if '' != logPath:
            self.__sinks.append(spdlog.rotating_file_sink_mt(logPath, 1024, 5))

        self.__logger = spdlog.SinkLogger(self.__name, self.__sinks)

        if level == Logger.TRACE:
            self.__logger.set_level(spdlog.LogLevel.TRACE)
        elif level == Logger.DEBUG:
            self.__logger.set_level(spdlog.LogLevel.DEBUG)
        elif level == Logger.INFO:
            self.__logger.set_level(spdlog.LogLevel.INFO)
        elif level == Logger.WARN:
            self.__logger.set_level(spdlog.LogLevel.WARN)
        elif level == Logger.ERR:
            self.__logger.set_level(spdlog.LogLevel.ERR)
        else:
            print('UNKNOWN log level {}'.format(level))

    def trace(self, instruction):
        self.__logger.trace(instruction)

    def debug(self, instruction):
        self.__logger.debug(instruction)

    def info(self, instruction):
        self.__logger.info(instruction)

    def warn(self, instruction):
        self.__logger.warn(instruction)

    def err(self, instruction):
        self.__logger.err(instruction)


g_logger = None

def setLogger(name, level, isConsole = False, logPath = ''):
    global g_logger
    g_logger = Logger(name, level, isConsole, logPath)
