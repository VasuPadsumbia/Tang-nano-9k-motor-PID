import socket
import sys
import time

FPGA_IP = "192.168.1.100"
PORT = 5005

def listen():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", PORT))
    print(f"Listening UDP on :{PORT}")
    while True:
        data, addr = s.recvfrom(2048)
        print(f"{addr}: {data!r} -> {data.decode(errors='ignore')}")

def send(msg):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(msg.encode(), (FPGA_IP, PORT))
    print("sent")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        listen()
    else:
        send(" ".join(sys.argv[1:]))
