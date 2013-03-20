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

def last_resort_ping(address, port):
    try:
        alivePeriod = 2
        while True:
            try:
                print("Pinging last resort on {}:{}".format(address,port))
                sock = socket.socket()
                sock.connect((address, port))
                sock.close()

            except socket.error as e:
                print("Pinging last resort failed {}".format(e))
                pass
            time.sleep(alivePeriod//2)
    
    except KeyboardInterrupt:
        s = 'WARNING: last_resort_ping stopped by keyboard interrupt\n'
    finally:
        s = 'last_resport_ping.py finished at %s\n' % str(time.time())
        sock.close()



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("address",default="10.0.0.2", nargs='?',help="address of the sub to ping")
    parser.add_argument("port",type=int,default=7777, nargs='?',help="port of the sub to ping")
    args = parser.parse_args()
    last_resort_ping(args.address, args.port)

