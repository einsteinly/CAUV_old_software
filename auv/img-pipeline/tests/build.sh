#! /bin/bash

g++ -I /societies/cauv/install/include/ -I /usr/include/opencv/ -I \
../.. -L /societies/cauv/install/lib/ -lcxcore -lcv \
-lhighgui -lboost_thread -lboost_serialization -lspread ../scheduler.cpp \
../node.cpp ../nodes/nodes.cpp ../../.obj/debug/common.o -lssrcspread $1.cpp -o $1 -Wall -g -DDEBUG

