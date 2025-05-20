import socket

computerIP = "192.168.0.109" # check wi-fi parameters
socketPort = 8888 #any unused port
EspIP = "192.168.0.106" # check in Arduino
EspPort = 2000 # check in Arduino


class UDP:
    def __init__(self):
        self.send_socket = socket.socket(socket.AF_INET,  # Internet
                                         socket.SOCK_DGRAM)  # UDP

    def send(self, mess, targetIP = EspIP, targetPort = EspPort):
        if isinstance(mess, str): # checking that mess type is string
            mess = mess.encode("ascii")
        else:
            raise Exception("mess type is ",type(mess), " not string")
        res = self.send_socket.sendto(mess, (targetIP, targetPort))
        print(mess, "sended")
