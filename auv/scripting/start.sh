#!/bin/sh

SCRIPTING_DIR=$(dirname $0)
#echo "Scripting dir is: $SCRIPTING_DIR"

export LD_LIBRARY_PATH="$SCRIPTING_DIR/cauv/:$LD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="$SCRIPTING_DIR/cauv/:$DYLD_LIBRARY_PATH"
export PYTHONPATH="$SCRIPTING_DIR:$SCRIPTING_DIR/../python-module/:$PYTHONPATH"

#echo "LD path: $LD_LIBRARY_PATH"
#echo "DYLD path: $DYLD_LIBRARY_PATH"

python2.7 -c "import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.sonar
import cauv.node

node = cauv.node.Node('py-start')
try:
    auv = control.AUV(node)
    sonar = cauv.sonar.Sonar(node)
    pl = pipeline.Model(node)

    from IPython.Shell import IPShellEmbed
    ipshell = IPShellEmbed()
    ipshell()
finally:
    node.stop()
"


