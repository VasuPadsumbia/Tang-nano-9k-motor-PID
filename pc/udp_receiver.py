import socket

PC_IP = "10.10.10.1"
FPGA_IP = "10.10.10.100"
PORT = 5005

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((PC_IP, 0))
s.settimeout(2.0)

while True:
    msg = input("send> ").encode()
    s.sendto(msg, (FPGA_IP, PORT))
    data, addr = s.recvfrom(2048)
    print("recv<", data.decode(errors="replace"), "from", addr)
