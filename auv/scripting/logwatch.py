#!/usr/bin/env python2.7

import argparse
import subprocess
import curses
import os
import time
import re

def run_screen_cmd(command):
    subprocess.check_call(["screen" , "-X"] + command)

parser = argparse.ArgumentParser(description = "Watch a directory for log files and show them in a screen session")

parser.add_argument('-d', '--dirs', help="Directories to watch", nargs = '+')

opts = parser.parse_args()

if os.getenv("STY") is None:
    raise RuntimeError("You must run this within a screen session!")

run_screen_cmd(['hardstatus', 'alwayslastline'])
run_screen_cmd(['hardstatus', 'string', '%{= kG}[ %{G}logview %{g}]'
                                      + '[%= %{=kw}%?%-Lw%?%{r}(%{W}%n*%f%t%?(%u)%?%{r})%{w}%?%+Lw%?%?%= %{g}]'
                                      + '[%{B}%Y-%m-%d %{W}%c %{g}]'
                                        ])
#these depend on the terminal
run_screen_cmd(['bindkey', '\033[5D', 'prev'])
run_screen_cmd(['bindkey', '\033[1;5D', 'prev'])
run_screen_cmd(['bindkey', '\03305D', 'prev'])
run_screen_cmd(['bindkey', '\033[5C', 'next'])
run_screen_cmd(['bindkey', '\033[1;5C', 'next'])
run_screen_cmd(['bindkey', '\03305C', 'next'])

#run_screen_cmd(['screen', '-t', 'test', 'htop'])

def extract_node_name(log_filename):
    #filename looks something like:
    #2012-05-13-21-14-56.399901-persist.py.log
    #                           |relevant|
    #                           |  part  |
    try:
        return re.findall('\d{4}(?:-\d{1,2}){5}\.\d{6}-(.+)\.log', log_filename)[0]
    except IndexError:
        return log_filename

def mtime(filename):
    return os.stat(filename).st_mtime

seen_logs = set()

current_nodes = {}

print("Less will be opened automatically in windows." + 
      "Press ctrl+C to scroll back/search and shift+F to follow the output again")

while True:
    #this is too clever for it's own good
    new_logs = [(os.path.join(d,f), seen_logs.add(f))[0]
                for d in opts.dirs 
                   for f in os.listdir(d)
                      if f.endswith('.log') and f not in seen_logs]

    new_logs.sort(key = mtime)

    new_nodes = {extract_node_name(os.path.split(f)[-1]): f for f in new_logs}

    for node_name in new_nodes:
        if node_name in current_nodes:
            run_screen_cmd(['at', node_name, 'kill'])
        run_screen_cmd(['screen', '-t', node_name, 'less', '+F', '-R', new_nodes[node_name]])

    time.sleep(1)
