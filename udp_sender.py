import socket
import time

receiver = ("10.10.10.2", 5000)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sequence = 0

while True:
    message = f"RPI_ETH_TEST sequence={sequence}"
    sock.sendto(message.encode(), receiver)
    print(f"TX #{sequence}: {message}")

    sequence += 1
    time.sleep(0.5)
