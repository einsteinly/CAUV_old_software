#! /bin/bash

SIZE=4000
REPEATS=10
#REPEATS=100000
SIZE_DEV=0

$1/common/supermess/cleanup_smemtest
$1/common/supermess/lib_test &
sleep 0.1
$1/common/supermess/smem_send_random $SIZE $REPEATS $SIZE_DEV
$1/common/supermess/lib_test stop
sleep 0.1
killall lib_test

