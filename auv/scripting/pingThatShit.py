#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import argparse
import time
import socket

def plog(s, f):
    f.write(s)
    f.flush()
    print s,

def pingThatShit(address, port):
    try:
        alivePeriod = 2
        while True:
            try:
                print("Pinging that shit on {}:{}".format(address,port))
                sock = socket.socket()
                sock.connect((address, port))
                sock.close()

            except socket.error as e:
                print("Pinging that shit fucked up! {}".format(e))
                pass
            time.sleep(alivePeriod//2)
    
    except KeyboardInterrupt:
        s = 'WARNING: Ping that shit stopped by keyboard interrupt\n'
    finally:
        s = 'pingThatShit.py finished at %s\n' % str(time.time())
        sock.close()



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("address",default="10.0.0.2", nargs='?',help="address of the shit to ping")
    parser.add_argument("port",type=int,default=7777, nargs='?',help="port of the shit to ping")
    args = parser.parse_args()
    pingThatShit(args.address, args.port)

