#! /bin/bash
set -e
sudo ./run.sh ./watch.py ./sessions/stage1.py --bin-dir=/cauv/builds/debinfo/bin/ --script-dir=/cauv/repos/software.hg/auv/scripting/ --core-dumps
