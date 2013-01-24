#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import os
import sys
import socket
import argparse

import cauv.node
import cauv.messaging as messaging

parser = argparse.ArgumentParser(description="Send control messages to watch.py")

parser.add_argument("action", choices = ['start', 'stop', 'restart'], help='action to take on process')
parser.add_argument("process", help="process name to control")
#parser.add_argument("--cmd", "-c", help="Command line to execute", nargs = argparse.REMAINDER, default = [])
parser.add_argument("--loud", help="Don't silence stdout", action = 'store_true')
parser.add_argument("--host", "-t", help="Host to execute on. * for all hosts", default = socket.gethostname())

args = parser.parse_args()

if not args.loud:
    #hacky hack hack
    null_fd = os.open("/dev/null", os.O_RDWR)
    os.dup2(null_fd, sys.stdout.fileno())

node = cauv.node.Node('watchctl.py')

action_map = {
    'start' : messaging.ProcessCommand.Start,
    'stop'  : messaging.ProcessCommand.Stop,
    'restart' : messaging.ProcessCommand.Restart,
}
node.send(messaging.ProcessControlMessage(action_map[args.action], args.host, args.process))
node.stop()
