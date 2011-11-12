#!/usr/bin/env python2.7

# Standard Library Modules
import argparse
import datetime
import time
import sys

# Third Party Modules

# CAUV Modules
import cauv
import cauv.node
import cauv.messaging as msg
from utils import CHIL
from utils.hacks import tdToFloatSeconds
from cauv.debug import debug, error, warning, info

Strptime_Fmt = '%d/%m/%y-%H:%M:%S'

tzero = datetime.datetime.now()
def relativeTime():
    return tdToFloatSeconds(datetime.datetime.now() - tzero)

def play(fname, node, tstart, rt_rate, fixed_rate):
    p = CHIL.Player(fname)
    try:
        tstart_float = float(tstart)
        p.setCursor(datetime.datetime(year=datetime.MINYEAR, month=1, day=1))
        tzero = p.timeOfNextMessage()
        p.setCursor(tzero + datetime.timedelta(seconds=tstart_float))
    except:
        p.setCursor(datetime.datetime.strptime(tstart, Strptime_Fmt))
    
    if rt_rate is not None:
        assert(fixed_rate is None)
        tstart_playback = relativeTime()
        try:
            while True:
                m, td = p.nextMessage()
                if m is None:
                    break
                if isinstance(m, str):
                    warning(m)
                    m = None
                time_to_sleep_for = tdToFloatSeconds(td) - rt_rate * (relativeTime() - tstart_playback)
                if time_to_sleep_for/rt_rate > 10:
                    warning('more than 10 seconds until next message will be sent (%gs)' %
                            (time_to_sleep_for/rt_rate))
                while time_to_sleep_for > 0:
                    sleep_step = min((time_to_sleep_for/rt_rate, 0.2))
                    time_to_sleep_for -= sleep_step*rt_rate
                    time.sleep(sleep_step)
                sys.stdout.write('.'); sys.stdout.flush()
                if m is not None:
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
                if isinstance(m, str):
                    warning(m)
                    m = None
                time_to_sleep_for = max(0,(tlast+1.0/fixed_rate)-tlast)
                time.sleep(time_to_sleep_for)
                sys.stdout.write('.'); sys.stdout.flush()
                if m is not None:
                    node.send(m)
        except Exception, e:
            error('error in playback: ' + str(e))
            raise
        debug('playback finished')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='play a CHIL log file')
    p.add_argument('file', metavar='FILE', type=str)
    p.add_argument('-s', '--start', dest='start_t', default="0",
        help='start time: offset seconds or absolute %s' % Strptime_Fmt)
    p.add_argument('-r', '--rate', dest='rate', type=str, default='x1',
        help='messages per second (x1 for real-time, 2 for 2 messages per second)')
    opts, unknown_args = p.parse_known_args()
    
    tstart = opts.start_t
    rt_rate = None
    fixed_rate = None
    if opts.rate.startswith('x'):
        rt_rate = float(opts.rate[1:])
    else:
        fixed_rate = float(opts.rate)
    
    node = cauv.node.Node('py-play',unknown_args)
    try:
        play(opts.file, node, tstart, rt_rate, fixed_rate)
    finally:
        node.stop()
