from base_socket import ConnectionBase

from base_socket import ConnectionBase
import threading
import time 

class Client(ConnectionBase):
    def __init__(self, mode, port):
        super().__init__(mode, port)

    def notify(self):
        print(self.buffer)

a = Client("client", "communication_port")
a.gather_info = input

a.start()