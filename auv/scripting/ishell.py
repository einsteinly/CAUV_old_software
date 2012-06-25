#!/usr/bin/env python2.7
import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.sonar
import cauv.node

node = cauv.node.Node('py-shell')
auv = control.AUV(node)
sonar = cauv.sonar.Sonar(node)
gemini = cauv.sonar.Gemini(node)
pl = pipeline.Model(node)


import IPython
if [int(v) for v in IPython.__version__.split('.')] < [0,11]:
    from IPython.Shell import IPShellEmbed as InteractiveShellEmbed #pylint:disable=E0611
else:
    from IPython.frontend.terminal.embed import InteractiveShellEmbed #pylint:disable=E0611

s = InteractiveShellEmbed()
s()
