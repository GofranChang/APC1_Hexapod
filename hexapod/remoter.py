import socketserver
import threading
import struct

class RemoterTCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client
    """

    def __init__(self, request, client_address, server, remoter):
        self.__remoter = remoter
        super().__init__(request, client_address, server)

    def handle(self):
        print("Got connection from", self.client_address)
        self.__remoter.on_connected()

        from_server_msglen = self.request.recv(4)
        unpack_len_msg = struct.unpack("i", from_server_msglen)[0]

        print('Recv {} byte msg'.format(unpack_len_msg))

        recv_msg_len = 0
        all_msg = b""
        while recv_msg_len < unpack_len_msg:
            every_recv_date = self.request.recv(1024)
            all_msg += every_recv_date                      #将每次接收到的数据进行拼接和统计
            recv_msg_len += len(every_recv_date)            #对每次接受到的数据进行累加

        self.__remoter.on_command(recv_msg_len, all_msg.decode('utf-8'))

    def finish(self):
        print('{} disconnect'.format(self.client_address[0]))
        self.__remoter.on_disconnected()

    @classmethod
    def Creator(cls, *args, **kwargs):
        def _HandlerCreator(request, client_address, server):
            cls(request, client_address, server, *args, **kwargs)
        return _HandlerCreator

class TcpRemote(object):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def __init__(self, port):
        self.__reset_all_cmd()

        self.__server = socketserver.TCPServer(('', port), RemoterTCPHandler.Creator(self))
        try:
            self.__server_thread = threading.Thread(target = self.__server.serve_forever)
            self.__server_thread.start()
        except KeyboardInterrupt:
            pass

        self.__is_conneted = False

    def on_command(self, len, msg):
        if not self.__is_conneted:
            self.__reset_all_cmd()
            return

        # if self.m_server.conn
        print('On command msg : "{}"'.format(msg))

    def on_connected(self):
        self.__is_conneted = True

    def on_disconnected(self):
        self.__is_conneted = False

    def is_connected(self):
        return self.__is_conneted

    def __reset_all_cmd(self):
        """
        Reset all command
        """
        self.__cmd = ''
        self.__command_x = 0
        self.__command_y = 0
        self.__command_r = 0

    def get_command(self):
        if not self.__is_conneted:
            self.__reset_all_cmd()

        cmd = self.__cmd
        command_x = self.__command_x
        command_y = self.__command_y
        command_r = self.__command_r

        return cmd, command_x, command_y, command_r

# if __name__ == "__main__":
#     remoter = HexapodRemoter(8999)