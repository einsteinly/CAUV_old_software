#!/usr/bin/env python2.7
import zmq
import os
import sys
import struct
import time

node_name = sys.argv[1]

c = zmq.Context(1)

vehicle_dir = '/tmp/cauv/red_herring'

sub = zmq.Socket(c,zmq.SUB)
sub_name = 'ipc://' + vehicle_dir + '/' + node_name + '.' + str(os.getpid())
sub.bind(sub_name)
sub.setsockopt(zmq.SUBSCRIBE, struct.pack('=LL',223,os.getpid()))
pub = zmq.Socket(c,zmq.XPUB)
connected = set()

def sendmsg(socket, msg_id, message):
    return socket.send(struct.pack('=L' + str(len(message)) + 's',msg_id,message))

def recvmsg(socket):
    _msg = socket.recv()
    return struct.unpack('=L' + str(len(_msg) - struct.calcsize('=L')) + 's',_msg)

def subscribe(socket, msg_id):
    return socket.setsockopt(zmq.SUBSCRIBE, struct.pack('=L',msg_id))

wait_for_pids = set([])
for socket in os.listdir(vehicle_dir):
    connect_string = 'ipc://' + vehicle_dir + '/' + socket
    pid = None
    try:
        pid = int(socket.rsplit('.',1)[-1])
        os.kill(pid,0)
    except ValueError:
        continue
    except OSError, e:
        if e.errno == 3: #process doesn't exist
            os.unlink(vehicle_dir + '/' + socket)
            continue
    if connect_string != sub_name:
        print(connect_string)
        pub.connect(connect_string)
        connected.add(connect_string)
        wait_for_pids.add(pid)

subscribe(sub, 432)
subscribe(sub, 43)

while wait_for_pids:
    msg = pub.recv()
    if len(msg) == struct.calcsize('=LL') + 1:
        dummy,pid = struct.unpack('=LL',msg [1:])
        print('pid ' + str(pid) + ' connected')
        wait_for_pids.discard(pid)

print('sending connect message')
sendmsg(pub, 432, sub_name)

while True:
    if sub.poll(500):
        msg = recvmsg(sub)
        if msg[0] == 432:
            if msg[1] not in connected:
                print('connecting to ' + msg[1])
                pub.connect(msg[1])
                connected.add(msg[1])
        if msg[0] == 43:
            print(msg[1])
    else:
        sendmsg(pub, 43, node_name)
