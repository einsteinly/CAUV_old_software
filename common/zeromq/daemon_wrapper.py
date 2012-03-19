#!/usr/bin/env python2.7

import subprocess
import time
import argparse

#!!! could/should be generated from messages-python
groups = [
'membership',
'debug',
'control',
'state',
'telemetry',
'image',
'sonarout',
'sonarctl',
'pipeline',
'processing',
'gui',
'pl_gui',
'mcb',
'pressure',
'ai',
'guiai',
'external',
'simulation'
]


aparser = argparse.ArgumentParser(description = 'run zmq daemons for all groups')
aparser.add_argument('-b','--daemon_binary',default = 'cauv-zmq_daemon')
args = aparser.parse_args()

daemon_bin = args.daemon_binary

daemons = []
class GroupDaemon:
    def __init__(self,name):
        self.name = name
        self.proc = None

    def run(self):
        self.proc = subprocess.Popen([daemon_bin,'-g',self.name])
    
    def running(self):
        if self.proc is None:
            return False
        self.proc.poll()
        if self.proc.returncode is not None:
            return False
        return True

daemons = [GroupDaemon(g) for g in groups]

while True:
    for daemon in daemons:
        if not daemon.running():
            print("daemon for group {} stopped. (re)starting".format(daemon.name))
            daemon.run()
            time.sleep(0.1)
    time.sleep(10)

