import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.10.10.2", 5000))
print("Ethernet 수신 대기 중", flush=True)

try:
    while True:
        data, sender = sock.recvfrom(2048)
        print(f"{sender}: {data.decode(errors='replace')}", flush=True)
except KeyboardInterrupt:
    pass
finally:
    sock.close()
