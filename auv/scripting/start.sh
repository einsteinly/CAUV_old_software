#!/bin/sh

SCRIPTING_DIR=$(dirname $(readlink -f $0))

export LD_LIBRARY_PATH="$SCRIPTING_DIR/cauv/:$LD_LIBRARY_PATH"
export PYTHON_PATH="$SCRIPTING_DIR:$PYTHONPATH"

python -i -c "import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.node

node = cauv.node.Node('py-start')
auv = control.AUV(node)
pl = pipeline.Model(node)

"


