import socket

UDP_IP = "192.168.0.107"
UDP_PORT = 2000
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP


UDP_IP1 = "192.168.0.109"
UDP_PORT1 = 8888
print(socket.gethostname())
sock_recv = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_recv.setblocking(0)
sock_recv.settimeout(0.1)
sock_recv.bind((UDP_IP1, UDP_PORT1))


while True:
    try:
        data, addr = sock_recv.recvfrom(1024) # buffer size is 1024 bytes
        res = sock.sendto(b'100,100/', (UDP_IP, UDP_PORT))
        print("received message: %s" % data.decode())
    except TimeoutError:
        pass
