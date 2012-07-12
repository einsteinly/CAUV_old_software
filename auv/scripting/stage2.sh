#! /bin/bash
set -e
./run.sh ./watch.py -s ./sessions/stage2.py --bin-dir=/cauv/builds/debinfo/bin/ --script-dir=/cauv/repos/software.hg/auv/scripting/ -K --core-dumps $@
