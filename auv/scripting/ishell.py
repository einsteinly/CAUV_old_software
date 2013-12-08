#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import cauv
import rospy
#import cauv.pipeline as pipeline
import cauv.control as control
#import cauv.sonar
#import cauv.node

rospy.init_node("py_shell")
auv = control.AUV()
#sonar = cauv.sonar.Sonar(node)
#gemini = cauv.sonar.Gemini(node)
#pl = pipeline.Model(node)

try:
    #All Crosby's fault... damn Mac users...
    import IPython
    old_ipython = [int(v) for v in IPython.__version__.split('.')] < [0,11]
except:
    old_ipython = False
if old_ipython:
    from IPython.Shell import IPShellEmbed as InteractiveShellEmbed #pylint: disable=E0611
else:
    from IPython.frontend.terminal.embed import InteractiveShellEmbed #pylint: disable=E0611

s = InteractiveShellEmbed()
s()
