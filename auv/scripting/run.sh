#!/bin/sh

SCRIPTING_DIR="$(dirname $0)"
LD_EXT_PATH="$SCRIPTING_DIR/../python-module/cauv/:$SCRIPTING_DIR/../python-module/"

export LD_LIBRARY_PATH="$LD_EXT_PATH:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="$LD_EXT_PATH:$DYLD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPTING_DIR:$SCRIPTING_DIR/../python-module/:$PYTHONPATH"

#echo $DYLD_LIBRARY_PATH

python $@

