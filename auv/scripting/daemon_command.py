#!/usr/bin/env python2.7

import zmq
import argparse
import sys
import os
import utils.zmqfuncs
import argparse

vehicle_name = 'red_herring'
ipc_dir = '/tmp/cauv/'

if os.getenv('CAUV_IPC_DIR') is not None:
    ipc_dir = os.getenv('CAUV_IPC_DIR')

if os.getenv('CAUV_VEHICLE_NAME') is not None:
    vehicle_name = os.getenv('CAUV_VEHICLE_NAME')

control_rel = 'daemon/control'

parser = argparse.ArgumentParser(description="Run a command as a daemon")
parser.add_argument('--dir','-d',help='Daemon control dir')
parser.add_argument('--vehicle','-v', help='Vehicle name')
parser.add_argument('--ipc-dir', '-i', help='Vehicle name')
parser.add_argument('commands', help='Commands to run', nargs = argparse.REMAINDER)

opts = parser.parse_args()

if opts.dir:
    arg = opts.dir
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
s.connect("ipc://{}/daemon/control".format(utils.zmqfuncs.get_vehicle_dir(opts.ipc_dir, opts.vehicle)))

if opts.commands:
    for command in opts.commands:
        s.send(command)
        print(' '.join(s.recv_multipart()))
else:
    import readline
    while True:
        command = raw_input(">")
        s.send(command)
        print(' '.join(s.recv_multipart()))
