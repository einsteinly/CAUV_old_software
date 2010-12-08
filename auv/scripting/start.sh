#!/bin/sh

SCRIPTING_DIR="$(pwd)"
LD_EXT_PATH="$SCRIPTING_DIR/../python-module/cauv/:$SCRIPTING_DIR/../python-module/"

export LD_LIBRARY_PATH="$LD_EXT_PATH:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="$LD_EXT_PATH:$DYLD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPTING_DIR:$SCRIPTING_DIR/../python-module/:$PYTHONPATH"

python -i -c "import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.node

node = cauv.node.Node('py-start')
auv = control.AUV(node)
pl = pipeline.Model(node)

"


