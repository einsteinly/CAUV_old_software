#!/usr/bin/env python2.7
import argparse
import os
import time
import sys
import socket

def plog(s, f):
    f.write(s)
    f.flush()
    print s,

def lastResort(port, kill_after_seconds):
    logf = open('last-resort.log', 'a')
    sock = socket.socket()
    sock.bind(("", port))
    sock.listen(5)
    alivePeriod = 2
    try:
        while True:
            try:
                sock.settimeout(alivePeriod)
                s = sock.accept()
                s[0].close()
                connected = True
            except socket.timeout:
                start_time = time.time() - alivePeriod
                s = '\nlastresort.py lost connection at %s\n' % str(start_time)
                plog(s, logf)
                did_warn_30 = False
                did_warn_10 = False
            
                while True:
                    sock.settimeout(0.5)
                    try:
                        s = sock.accept()
                        s[0].close()
                        s = '\nlastresort.py reconnected at %s! Yay!\n' % str(start_time)
                        plog(s, logf)
                        break
                    except socket.timeout:
                        pass
                    
                    dt = time.time() - start_time
                    if dt > kill_after_seconds - 30 and not did_warn_30:
                        s = 'WARNING: will start killing things in %s seconds\n' % str(kill_after_seconds - dt)
                        did_warn_30 = True
                        plog(s, logf)
                    if dt > kill_after_seconds - 10 and not did_warn_10:
                        s = 'WARNING: will start killing things in %s seconds\n' % str(kill_after_seconds - dt)
                        did_warn_10 = True
                        plog(s, logf)
                    if dt > kill_after_seconds:
                        while True:
                            s = 'DIE control, DIE!\n'
                            plog(s, logf) 
                            os.system('killall -s 9 control')
                            os.system('killall -s 9 controld')
                            os.system('killall -s 9 cauv-control')
                            os.system('killall -s 9 cauv-controld')
                            os.system('killall -s 9 cauv-controlv2')
                            os.system('killall -s 9 mcb-bridge')
                            os.system('killall -s 9 mcb-bridged')
                            time.sleep(0.8)
    
    except KeyboardInterrupt:
        s = 'WARNING: Last resort stopped by keyboard interrupt\n'
        plog(s, logf)
    finally:
        s = 'lastresort.py finished at %s\n' % str(time.time())
        plog(s, logf)
        logf.close()
        sock.close()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(help='usage: sudo last-resort.py PORT TIME_IN_SECONDS')
    parser.parse_args()
    if len(args) != 2:
        parser.print_help()
        sys.exit(1)
    port = int(args[0])
    tf = float(args[1])
    lastResort(port,tf)

