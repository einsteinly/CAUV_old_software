#!/bin/sh

SCRIPTING_DIR="$(pwd)"
#echo "Scripting dir is: $SCRIPTING_DIR"

export LD_LIBRARY_PATH="$SCRIPTING_DIR/../python-module/cauv/:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="$SCRIPTING_DIR/../python-module/cauv/:$DYLD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPTING_DIR:$SCRIPTING_DIR/../python-module/:$PYTHONPATH"

#echo "LD path: $LD_LIBRARY_PATH"
#echo "DYLD path: $DYLD_LIBRARY_PATH"

python -i -c "import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.node

node = cauv.node.Node('py-start')
auv = control.AUV(node)
pl = pipeline.Model(node)

"


