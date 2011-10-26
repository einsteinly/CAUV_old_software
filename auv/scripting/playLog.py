#!/usr/bin/env python

# Standard Library Modules
import argparse
import datetime
import time
import sys

# Third Party Modules

# CAUV Modules
import cauv
import cauv.node
from utils import CHIL
from utils.hacks import tdToFloatSeconds
from cauv.debug import debug, error, warning, info

tzero = datetime.datetime.now()
def relativeTime():
    return tdToFloatSeconds(datetime.datetime.now() - tzero)

def play(fname, node, tstart, rt_rate, fixed_rate):
    p = CHIL.Player(fname)
    p.setCursor(datetime.datetime(year=datetime.MINYEAR, month=1, day=1))
    tzero = p.timeOfNextMessage()
    p.setCursor(tzero + datetime.timedelta(seconds=tstart))
    
    if rt_rate is not None:
        assert(fixed_rate is None)
        tstart_playback = relativeTime()
        try:
            while True:
                m, td = p.nextMessage()
                if m is None:
                    break
                time_to_sleep_for = tdToFloatSeconds(td) - rt_rate * (relativeTime() - tstart_playback)
                if time_to_sleep_for/rt_rate > 10:
                    warning('more than 10 seconds until next message will be sent (%gs)' %
                            (time_to_sleep_for/rt_rate))
                while time_to_sleep_for > 0:
                    sleep_step = min((time_to_sleep_for/rt_rate, 0.2))
                    time_to_sleep_for -= sleep_step*rt_rate
                    time.sleep(sleep_step)
                sys.stdout.write('.'); sys.stdout.flush()
                #print m.__class__.__name__
                node.send(m)
        except Exception, e:
            error('error in playback: ' + str(e))
            raise
        debug('playback finished')
    else:
        assert(fixed_rate is not None)
        try:
            while True:
                tlast = relativeTime() 
                m, td = p.nextMessage()
                if m is None:
                    break
                time_to_sleep_for = max(0,(tlast+1.0/fixed_rate)-tlast)
                time.sleep(time_to_sleep_for)
                sys.stdout.write('.'); sys.stdout.flush()
                #print m.__class__.__name__
                node.send(m)
        except Exception, e:
            error('error in playback: ' + str(e))
            raise
        debug('playback finished')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='play a CHIL log file')
    p.add_argument('file', metavar='FILE', type=str, nargs=1)
    p.add_argument('-s', '--start', dest='start_t', type=float, default=0.0,
        help='start time offset (seconds)')
    p.add_argument('-r', '--rate', dest='rate', type=str, default='x1',
        help='messages per second (x1 for real-time, 2 for 2 messages per second)')
    args = p.parse_args()
    
    tstart = args.start_t
    rt_rate = None
    fixed_rate = None
    if args.rate.startswith('x'):
        rt_rate = float(args.rate[1:])
    else:
        fixed_rate = float(args.rate)
    
    node = cauv.node.Node('py-sim')
    try:
        print args.file
        play(args.file[0], node, tstart, rt_rate, fixed_rate)
    finally:
        node.stop()
