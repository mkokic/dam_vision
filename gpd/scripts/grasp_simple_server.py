#
# @contactrika
#
# A very simple server to get grasp parameters from abother machine.
#
import argparse
import os
import socket
import sys

import numpy as np


SERVER_IP = '130.237.218.124'


def get_args():
    parser = argparse.ArgumentParser(description="GraspServer")
    parser.add_argument('--address', type=str, default=None,
                        help='Server TCP Port')
    parser.add_argument('--port', type=int, default=10000,
                        help='Server TCP Port')
    args = parser.parse_args()
    return args


def get_grasp_from_file(fname):
    #data = np.loadtxt(fname)
    data = open(fname, 'rb').read()
    print('data', data)
    return data


def main_server():
    args = get_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # init TCP socket
    server_address = (SERVER_IP, args.port)  # bind the socket to the port
    print('starting up on %s port %s' % server_address)
    sock.bind(server_address)
    sock.listen(1)
    bufsize = 1024
    while True:
        print('waiting for a connection')
        connection, client_address = sock.accept()
        try:
            print('connection from', client_address)
            fname_bytes = connection.recv(bufsize)
            fname = fname_bytes.decode('utf-8')
            print('received "%s"' % fname)
            grasp_bytes = get_grasp_from_file(fname)
            connection.sendall(grasp_bytes)
            print('done with client', client_address)
        finally:
            connection.close()  # clean up


if __name__ == "__main__":
    main_server()
