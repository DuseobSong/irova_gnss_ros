#!/usr/bin/env python3

import socket
import os

stream = os.popen('hostname -I | tr -d " "')
ip_addr = stream.read().split('.')

UDP_IP = "{}.{}.{}.255".format(ip_addr[0], ip_addr[1], ip_addr[2])
UDP_PORT = 20000

print('[GNSS] Listening ...')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((UDP_IP, UDP_PORT))

while True:
    try:
        data, addr = sock.recvfrom(1024)
        print('[GNSS] received <{}>: {}'.format(addr[1], data))
    except KeyboardInterrupt:
        break