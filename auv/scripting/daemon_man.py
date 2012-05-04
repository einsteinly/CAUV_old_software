#!/usr/bin/env python2.7

import cauv.node
import cauv.messaging as messaging
from cauv.debug import debug, info, warning, error
import argparse
import socket
import time
import utils.zmqfuncs
import zmq
import os
import sys
import random
import re

parser = argparse.ArgumentParser(description = 'Manage vehicle_daemon network connections')
parser.add_argument('--peer', '-s', help='first peer to connect to')
parser.add_argument('--peer-port', '-p', help='port of first peer to connect to',
                    type=int, default=4444)
parser.add_argument('--port', '-b', help='port to bind to',
                    type=int, default=4444)
parser.add_argument('--vehicle', '-v', help='daemon vehicle name')
parser.add_argument('--ipc-dir', '-i', help='ipc directory')
parser.add_argument('--pgm-addr', '-a', help='(E)PGM multicast address to use',
                    default='239.192.1.1:5555')
parser.add_argument('--no-pgm', '-n', help='Don\'t try to use PGM (currently default)',
                    action='store_true')

opts, args = parser.parse_known_args()
opts.no_pgm = True

# hack hack hackity hack. zmq asserts if the pgm address passed to it is
# invalid, instead of returning an error code or something...
# so, to check the address, we fork, try to connect, and then check the return
# code...
def check_pgm_addr(addr):
    if addr.find('127.0.0.1') != -1:
        return False
    pid = os.fork()
    if pid > 0:
        r_pid, ret = os.wait()
        if ret != 0:
            warning("PGM address {} is invalid!".format(addr))
            return False
        else:
            return True
    else:
        #quiet things down so fatal-looking error messages don't confuse people
        try:
            null_fd = os.open("/dev/null", os.O_RDWR)
            os.dup2(null_fd, sys.stdin.fileno())
            os.dup2(null_fd, sys.stdout.fileno())
            os.dup2(null_fd, sys.stderr.fileno())
            zmq_context = zmq.Context(1)
            s = zmq.Socket(c, zmq.PUB)
            s.connect(addr)
            s.setsockopt(zmq.LINGER,200)
            s.send('TESTING')
            zmq_context.term()
        #should _always_ exit - bad things will probably happen otherwise
        except:
            os._exit(1)
        finally:
            os._exit(0)

def normalise_addr(addr, port):
    try:
        return socket.getaddrinfo(addr, port, socket.AF_INET, socket.SOCK_STREAM)[0][4]
    except socket.gaierror as e:
        error("error looking up addr {}:{}; {}".format(addr, port, e))

def normalise_conn_str(conn):
    if conn.startswith('tcp://'):
        addr, port = conn.split('://')[1].split(':')
        port = int(port)
        norm_addr = normalise_addr(addr, port)
        if norm_addr is None:
            return None
        return 'tcp://{}:{}'.format(*norm_addr)
    else:
        return conn

class connObserver(messaging.MessageObserver):
    def __init__(self, node, port, ip, daemon_control):
        self.port = port
        self.node = node
        self.ip = ip
        self.d_ctrl = daemon_control
        self.found_peer = False

        self.daemon_id = self.d_ctrl.run_cmd('ID')['id']
        self.daemon_connections = set((normalise_conn_str(str(c)) for c in 
                                       self.d_ctrl.run_cmd('CONNECTIONS')['net_xpub']['connections']))
        debug("{}".format(self.daemon_connections))

        messaging.MessageObserver.__init__(self)
        self.node.addObserver(self)

    def onDaemonAnnounceMessage(self, message):
        debug(str(message))
        if message.daemon_id != self.daemon_id:
            self.found_peer = True
        self.try_connect(message.connect_string)
        self.send_connections()

    def onDaemonConnectionsMessage(self, message):
        debug(str(message))
        for conn_string in message.connect_strings:
            self.found_peer = True
            self.try_connect(conn_string)

    def onDaemonConnectedMessage(self, message):
        debug(str(message))
        if message.daemon_id != self.daemon_id:
            self.found_peer = True
            self.announce()
            self.send_connections()

    def send_connections(self):
        debug("sending connections dictionary")
        self.node.send(messaging.DaemonConnectionsMessage(list(self.daemon_connections)))

    def get_conn_str(self):
        return 'tcp://{}:{}'.format(self.ip, self.port)

    def announce(self):
        debug("announcing presense")
        self.node.send(messaging.DaemonAnnounceMessage(self.daemon_id,self.get_conn_str()))

    def try_connect(self, conn_string):
        n_conn = normalise_conn_str(conn_string)
        if n_conn == normalise_conn_str(self.get_conn_str()):
            debug('not connecting to self ({})'.format(self.get_conn_str()))
        elif n_conn not in self.daemon_connections: 
            debug("connecting to {}".format(conn_string))
            self.d_ctrl.run_cmd("CONNECT NET_XPUB {}".format(conn_string))
            self.daemon_connections.add(n_conn)
        else:
            debug("already connected to {} ({})".format(conn_string, n_conn))


#figure out IP address
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((opts.peer, opts.peer_port))
ip_str = s.getsockname()[0]
debug("Our IP address is {}".format(ip_str))
s.close()

pgm_connect_str = 'epgm://{};{}'.format(ip_str, opts.pgm_addr)
if opts.no_pgm:
    pgm_addr_valid = False
else:
    pgm_addr_valid = check_pgm_addr(pgm_connect_str)

d_ctrl = utils.zmqfuncs.DaemonControl(opts.vehicle, opts.ipc_dir)
pid = d_ctrl.run_cmd("PID")['pid']
debug("Connected to daemon control with PID {}".format(pid))
d_ctrl.run_cmd("BIND NET_XSUB tcp://*:{}".format(opts.port))
try:
    n = cauv.node.Node('daemon-man', args)
    obs = connObserver(n, opts.port, ip_str, d_ctrl)
    n.subMessage(messaging.DaemonAnnounceMessage())
    n.subMessage(messaging.DaemonConnectionsMessage())
    n.subMessage(messaging.DaemonConnectedMessage())
    if opts.peer:
        obs.try_connect('tcp://{}:{}'.format(opts.peer, opts.peer_port))
    # there's a race condition with starting up the node, so send afterwards as
    # well
    obs.announce()
    obs.send_connections()

    if pgm_addr_valid:
        debug('joining PGM stream {}'.format(pgm_connect_str))
        broadcast_skt = zmq.Socket(zmq_context, zmq.PUB)
        broadcast_skt.connect(pgm_connect_str)
        receive_skt = zmq.Socket(zmq_context, zmq.SUB)
        receive_skt.connect(pgm_connect_str)
        poller = zmq.Poller()
        p.register(receive_skt, zmq.POLLIN)
        while True:
            skt = p.poll(1000)
    else:
        while True:
            time.sleep(1)
            if not obs.found_peer:
                obs.announce()
                obs.send_connections()
finally:
    time.sleep(0.1)
    n.stop()
