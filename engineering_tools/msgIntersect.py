## Receive and Forward SAE J2735 Message Payload

import socket
from time import sleep
from binascii import unhexlify


def send(msg, ip_send, port_send):
    sk_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk_send.sendto(msg, (ip_send, port_send))
    # print("Sent: ", msg)
    sk_send.close()

def listen(ip_listen, port_listen):
    listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_socket.bind((ip_listen, port_listen))

    return listen_socket


def main():
    # declarations
    print('Starting intersect.')
    ip_listen = '127.0.0.1'
    ip_send = '192.168.0.146'
    port_listen = 1516  # listen to Immediate Forward Plugin
    port_send = 5398    # send to CARMA

    sk_listen = listen(ip_listen, port_listen)

    print("Waiting to receive data\n")
    while(1):
        data = sk_listen.recvfrom(10000)[0]
        data = data.decode('ascii')
        # print(data)

        idx = data.find("Payload=")
        payload = data[idx+8:-1]
        encoded = payload.encode('utf-8')
        msgBytes = unhexlify(encoded)

        # send
        send(msgBytes, ip_send, port_send)
        sleep(0.1)

if __name__=="__main__":
    main()
