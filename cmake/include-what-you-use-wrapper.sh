#!/bin/bash
# cmake expects whatever compiler it's given to work, 
# and since iwyu just checks what it needs to and then exits
# with an error code, cmake will barf. To fix this, this script 
# acts like clang but also calls iwyu. it could call gcc but that
# causes problems because cmake feeds slightly different flags to
# clang and gcc so the build will fail about halfway through.
#
# to use:
# CC=clang CXX=software.hg/cmake/include-what-you-use-wrapper.sh cmake [args]
# IWYU_OUTPUT_DIR=`pwd` make

# the version of clang built against can't find some directories, so they're added here
iwyu_flags="-I /usr/include/include-what-you-use $@"

#enable logging info to one file
if [ -z "$IWYU_OUTPUT_DIR" ]
then
    include-what-you-use $iwyu_flags
else
    echo logging to $IWYU_OUTPUT_DIR/include-what-you-use.log
    include-what-you-use $iwyu_flags &>> "$IWYU_OUTPUT_DIR/include-what-you-use.log"
    errr=$?
    if [ $errr -gt 127 ]
    then
        echo exited abnormally, saving errors to $IWYU_OUTPUT_DIR
        echo exited with code $errr >> $IWYU_OUTPUT_DIR/iwyu-errors.log
        echo command: >> $IWYU_OUTPUT_DIR/iwyu-errors.log
        echo include-what-you-use $iwyu_flags >> $IWYU_OUTPUT_DIR/iwyu-errors.log
    fi
fi
clang++ $@
exit $?
