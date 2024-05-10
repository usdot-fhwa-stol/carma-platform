## Receive and Forward SAE J2735 Message Payload
# This script receives messages sent by V2X Hub, strips the Active Message File header, and forwards only the message/payload to CARMA Messenger.
# The Active Message File is an ASCII-encoded hex string that is required by RSUs. Most other applications require only the message payload
# found at the end of the file, encoded as a UTF-8 hex string. 

# The script can be run on either PC running V2X Hub or CARMA Messenger. The following configurations must be made:
# ip_listen, port_listen: IPv4 Address and Port where messages will be received. Must match IP:Port set in Immediate Forward Plugin.
# ip_send, port_send: IPv4 Address and Port where payload will be sent. Must match IP, Port set in carma-cohda-dsrc-driver dsrc_driver/config/params.yaml.
# This script assumes both V2X Hub and CARMA Messenger are running on either the same PC or two separate PCs within the same local network. 


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
    port_send = 5398    # send to CARMA listening_port

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
