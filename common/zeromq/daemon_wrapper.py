#!/usr/bin/env python2.7

import subprocess
import time

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

processes = []

for group in groups:
    proc = subprocess.Popen(['./zmq_daemon','-g',group])

try:
    while True:
        time.sleep(10)
except:
    for proc in processes:
        proc.terminate()
