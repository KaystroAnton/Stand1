import socket

computerIP = "192.168.0.109" # check wi-fi parameters
socketPort = 8888 #any unused port

recv_socket = socket.socket(socket.AF_INET,  # Internet
                                         socket.SOCK_DGRAM)  # UDP
recv_socket.bind((computerIP, socketPort))
while True:
    data, addr = recv_socket.recvfrom(1024)  # buffer size is 1024 bytes
    print("received message: %s" % data.decode())
