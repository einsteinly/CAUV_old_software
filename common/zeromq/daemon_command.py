#!/usr/bin/env python2.7

import zmq
import argparse
import readline
import sys
import os

vehicle_name = 'red_herring'
ipc_dir = '/tmp/cauv/'

if os.getenv('CAUV_IPC_DIR') is not None:
    ipc_dir = os.getenv('CAUV_IPC_DIR')

if os.getenv('CAUV_VEHICLE_NAME') is not None:
    vehicle_name = os.getenv('CAUV_VEHICLE_NAME')

control_rel = 'daemon/control'

if len(sys.argv) > 1:
    arg = sys.argv[1]
    if 'control' in os.listdir(arg):
        ipc_dir = arg
        control_rel = 'control'
        vehicle_name = ''
    elif 'daemon' in os.listdir(arg):
        ipc_dir = arg
        vehicle_name = ''
    elif vehicle_name in os.listdir(arg):
        ipc_dir = arg
    else:
        print 'could not find control socket in specified area'
        exit(1)


c = zmq.Context(1)
s = zmq.Socket(c, zmq.REQ)
s.connect("ipc://" + ipc_dir + '/' + vehicle_name + '/' + control_rel)

while True:
    command = raw_input(">")
    s.send(command)
    print(' '.join(s.recv_multipart()))
