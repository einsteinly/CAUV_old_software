#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import utils.daemon
import sys
import os
import argparse
import functools

def exec_task(args):
    if not args.loud:
        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())
    os.chdir(args.dir)
    os.execlp(args.command[0], *args.command)

parser = argparse.ArgumentParser(description="Run a command as a daemon")
parser.add_argument('--loud', '-l', help='Show output of command on terminal (rather annoying to stop)',
                    action = 'store_true')
parser.add_argument('--dir', '-d', help='Run command in directory', default='/')
#undocumented nargs value
parser.add_argument('command', help='Command to run', nargs = argparse.REMAINDER)

args = parser.parse_args()
if not args.command:
    parser.print_help()
    sys.exit(1)
pid = utils.daemon.spawnDaemon(functools.partial(exec_task,args))
print('command run with PID {}'.format(pid))
