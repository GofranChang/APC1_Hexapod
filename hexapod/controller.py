from tcp_remoter import TcpRemote

class HexapodController(object):
    def __init__(self, type):
        if 'TcpRemoter' == type:
            self.__backend = TcpRemote(8998)

        pass

    def get_command(self):
        return self.__backend.get_command()

