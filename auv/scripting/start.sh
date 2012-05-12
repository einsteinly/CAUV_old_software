#!/bin/sh

SCRIPTING_DIR=$(dirname $0)
#echo "Scripting dir is: $SCRIPTING_DIR"

export LD_LIBRARY_PATH="$SCRIPTING_DIR/cauv/:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="$SCRIPTING_DIR/cauv/:$DYLD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPTING_DIR:$SCRIPTING_DIR/../python-module/:$PYTHONPATH"

#echo "LD path: $LD_LIBRARY_PATH"
#echo "DYLD path: $DYLD_LIBRARY_PATH"

python2.7 ${SCRIPTING_DIR}/ishell.py
