import socket
import logging
import json
import time 
import threading

logging.basicConfig(level=logging.INFO, format=':%(message)s')

class ConnectionBase:
    def __init__(self, mode, port):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        with open(r"communication/scripts/com_vars.json", "r") as vars:
            self.sok_vars = json.load(vars)
            self.host = self.sok_vars['server_ip']
            self.port = self.sok_vars[port]
            self.exit_pswrd = self.sok_vars['exit_password']
        self.stop_threads = False
        self.end_instance  = False
        self.mode = mode  
        #self.notify = None
        #self.gather_info = None
        self.buffer = None 
        
    def kill(self):
        self.end_instance = True

    def bind_socket_ifneeded(self):
        pass
    
    def create_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.logger.debug(f"Socket created at {self.host}:{self.port}")
        self.bind_socket_ifneeded()
        
    def connect(self):
        self.tries = 0 
        self.create_socket()
        while True:
            self.logger.debug(f"Attempt {self.tries}")
            try:
                if self.mode == "server":
                    self.socket.listen(0)
                    self.client, self.addr = self.socket.accept()
                    self.logger.info(f"Client conected at {self.addr}")
                    self.stop_threads = False
                    break
                
                elif self.mode == "client":
                    self.socket.connect((self.host, self.port))
                    self.logger.info(f"Conected to server{self.host}:{self.port}")
                    break

            except Exception as e:
                self.logger.error(f"Error connecting: {e}")
                self.tries += 1
                if self.mode == "server":
                    self.client.close()
                self.socket.close()
                time.sleep(1)
                if self.tries > 1:
                    self.logger.error(f"Limit of attempts reached")
                    break

            except Exception as e:
                self.logger.error(f"Error connecting: {e}")
                self.tries += 1
                if self.mode == "server":
                    self.client.close()
                self.socket.close()
                continue
                

    def read(self):
        #remember to create the notify method in the child class
        if self.mode == "server":
            read_from = self.client
        else:
            read_from = self.socket

        while not self.stop_threads:
            try:
                message = read_from.recv(2048).decode()
                if not message:
                    continue

                if message == self.exit_pswrd:
                    self.logger.warning("Exit password received")
                    self.buffer = message
                    self.notify()
                    self.kill()
                    self.stop_threads = True

                self.buffer = message
                self.notify()
                self.logger.debug(f"Received: {message}")    
                message = ''
            except Exception as e:
                self.logger.warning(f"Error at reading: {e}")
                self.stop_threads = True

    def send(self):
        #remember to create the gather_info method in the child class
        if self.mode == "server":
            send_to = self.client
        else:
            send_to = self.socket

        while not self.stop_threads:
            try:
                message = self.gather_info().encode()
                send_to.send(message)
            except Exception as e:
                self.logger.error(f"Error at sending: {e}")
                self.stop_threads = True

    def start(self):
        conections = 0
        while not self.end_instance:  
            try:
                self.logger.info(f"Starting {self.mode}, attempt {conections}")
                conections += 1
                self.connect()
                t1 = threading.Thread(target=self.read)
                t2 = threading.Thread(target=self.send)
                t1.start()
                t2.start()
                t1.join()
                t2.join()
            except Exception as e:
                self.logger.error(f"Error en los hilos: {e}")
            finally:
                self.logger.warning("Threads finished, closing socket")
                if self.mode == "server":
                    self.client.close()  
                self.socket.close()  
                time.sleep(2)  
