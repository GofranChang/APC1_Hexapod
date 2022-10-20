import imp
from socketserver import BaseRequestHandler, TCPServer
import json
import struct
import threading
from warnings import catch_warnings

class RemoteMsgHandler(BaseRequestHandler):
    def __init__(self, request, client_address, server, remoter):
        self.__remoter = remoter
        super().__init__(request, client_address, server)

    def handle(self):
        print("Got connection from", self.client_address)
        self.__remoter.on_connected()

        while True:
            from_server_msglen = self.request.recv(4)
            if not from_server_msglen:
                break

            unpack_len_msg = struct.unpack('i', from_server_msglen)[0]
            print('Recv {} byte msg'.format(unpack_len_msg))

            recv_msg_len = 0
            all_msg = b''
            while recv_msg_len < unpack_len_msg:
                unrecv_len = unpack_len_msg - recv_msg_len
                if unrecv_len >= 1024:
                    cur_recv_len = 1024
                else:
                    cur_recv_len = unrecv_len

                every_recv_date = self.request.recv(cur_recv_len)
                all_msg += every_recv_date                      #将每次接收到的数据进行拼接和统计
                recv_msg_len += len(every_recv_date)            #对每次接受到的数据进行累加

            print('xxxx . {}'.format(all_msg))
            uuid = self.__remoter.on_command(all_msg.decode('utf-8'))
            print('Get uuid : {}'.format(uuid))

            # uuid_len = len(uuid)
            # msg_len_stru = struct.pack('i', uuid_len)
            # self.request.send(msg_len_stru)
            # self.request.send(('{}\r\n'.format(uuid)).encode(encoding = 'utf-8'))

        print('Connection close')
        self.__remoter.on_disconnected()

    # def handle(self):
    #     print("Got connection from", self.client_address)
    #     while True:
    #         msg = self.request.recv(8192)
    #         if not msg:
    #             break

    #         print(msg)
    #         self.request.send('OK\r\n'.encode(encoding = 'utf-8'))
    #         self.__remoter.on_command(msg.decode('utf-8'))

    #     print('Connection close')

    @classmethod
    def Creator(cls, *args, **kwargs):
        def _HandlerCreator(request, client_address, server):
            cls(request, client_address, server, *args, **kwargs)
        return _HandlerCreator

class TcpRemote(object):
    def __init__(self, port):
        self._serv = TCPServer(('', port), RemoteMsgHandler.Creator(self))

        self.__reset_cmd()
        try:
            self.__servo_thread = threading.Thread(target = self._serv.serve_forever)
            self.__servo_thread.start()
        except:
            pass
        # self._serv.serve_forever()

        self.__is_conneted = False

    def disconnect(self):
        return

    def on_command(self, msg):
        print('>>>>>>>>>>>>>>>>>>>>>>>>>.')
        self.__reset_cmd()

        print('On command :', msg)
        try:
            json_cmd = json.loads(msg)
        except:
            return 'ERROR'

        self.__cmd = json_cmd['command']

        if 'move' == self.__cmd:
            self.__command_x = json_cmd['commandX']
            self.__command_y = json_cmd['commandY']
            self.__command_r = json_cmd['commandR']

        if 'head' == self.__cmd:
            self.__head_x = json_cmd['commandX']
            self.__head_y = json_cmd['commandY']

        return json_cmd['UUID']

    def __reset_cmd(self):
        self.__cmd = ''
        self.__command_x = 0
        self.__command_y = 0
        self.__command_r = 0
        self.__head_x = 0
        self.__head_y = 0

    def on_connected(self):
        self.__is_conneted = True

    def on_disconnected(self):
        self.__is_conneted = False

    def is_connected(self):
        return self.__is_conneted

    def get_command(self):
        if not self.__is_conneted:
            self.__reset_cmd()

        cmd = self.__cmd
        command_x = self.__command_x
        command_y = self.__command_y
        command_r = self.__command_r
        head_x = self.__head_x
        head_y = self.__head_y

        return cmd, command_x, command_y, command_r, head_x, head_y

# if __name__ == '__main__':
#     remote = TcpRemote(8999)
#     while True:
#         # print(11111)
#         pass