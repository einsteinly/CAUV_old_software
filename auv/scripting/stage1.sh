#! /bin/bash
set -e
sudo chmod 777 /dev/ttyS0
./run.sh ./watch.py -s ./sessions/stage1.py --bin-dir=/cauv/builds/debinfo/bin/ --script-dir=/cauv/repos/software.hg/auv/scripting/ --core-dumps $1

