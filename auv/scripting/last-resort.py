#! /usr/bin/env python
import optparse
import os
import time
import sys

def plog(s, f):
    f.write(s)
    f.flush()
    print s,

def lastResort(kill_after_seconds):
    start_time = time.time()
    logf = open('last-resort.log', 'a')
    
    s = '\nlastresort.py started at %s\n' % str(time.time())
    plog(s, logf)
    try:
        did_warn_30 = False
        did_warn_10 = False
        while True:
            dt = time.time() - start_time
            if dt > kill_after_seconds:
                break
            if dt > kill_after_seconds - 30 and not did_warn_30:
                s = 'WARNING: will start killing things in %s seconds\n' % str(kill_after_seconds - dt)
                did_warn_30 = True
                plog(s, logf)
            elif dt > kill_after_seconds - 10 and not did_warn_10:
                s = 'WARNING: will start killing things in %s seconds\n' % str(kill_after_seconds - dt)
                did_warn_10 = True
                plog(s, logf)
            time.sleep(0.5)
        s = 'DIE control, DIE!\n'
        
        os.system('killall -s 9 control')
        os.system('killall -s 9 cauv-control')
        os.system('killall -s 9 cauv-controld')
        os.system('killall -s 9 cauv-controlv2')
    except KeyboardInterrupt:
        s = 'WARNING: Last resort stopped by keyboard interrupt\n'
        plog(s, logf)
    finally:
        s = 'lastresort.py finished at %s\n' % str(time.time)
        plog(s, logf)
        logf.close()



if __name__ == '__main__':
    parser = optparse.OptionParser(usage='usage: sudo %prog TIME_IN_SECONDS')
    opts, args = parser.parse_args()
    if len(args) != 1:
        parser.print_help()
        sys.exit(1)
    tf = float(args[0])
    lastResort(tf)

