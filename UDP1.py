import socket
import string

from numpy.f2py.auxfuncs import throw_error

computerIP = "192.168.0.109" # check wi-fi parameters
socketPort = 8888 #any unused port
EspIP = "192.168.0.101" # check in Arduino
EspPort = 2000 # check in Arduino


class UDP:
    def __init__(self, ip = computerIP, port = socketPort):
        self.send_socket = socket.socket(socket.AF_INET,  # Internet
                                         socket.SOCK_DGRAM)  # UDP
        #self.recv_socket = socket.socket(socket.AF_INET,  # Internet
                                         #socket.SOCK_DGRAM)  # UDP
        #self.recv_socket.setblocking(False)
        #self.recv_socket.settimeout(0.1)
        #self.recv_socket.bind((ip, port))

    def send(self, mess, targetIP = EspIP, targetPort = EspPort):
        if isinstance(mess, str): # checking that mess type is string
            mess = mess.encode("ascii")
        else:
            raise Exception("mess type is ",type(mess), " not string")
        res = self.send_socket.sendto(mess, (targetIP, targetPort))
        print(mess, "sended")

    def listen(self): #for test
        while True:
            try:
                data, addr = self.recv_socket.recvfrom(1024)  # buffer size is 1024 bytes
                print("received message: %s" % data.decode())
            except TimeoutError:
                pass
