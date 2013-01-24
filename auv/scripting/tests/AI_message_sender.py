#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from AI_classes import aiProcess

from IPython.frontend.terminal.embed import InteractiveShellEmbed

ainode = aiProcess('ext_sender')

# FIXME Obvious.
try:
    ipython_version_is_jameses_shitty_one = [int(v) for v in IPython.__version__.split('.')] < [0,11]
except:
    ipython_version_is_jameses_shitty_one = False
if ipython_version_is_jameses_shitty_one:
    from IPython.Shell import IPShellEmbed as InteractiveShellEmbed #pylint: disable=E0611
else:
    from IPython.frontend.terminal.embed import InteractiveShellEmbed #pylint: disable=E0611

ipshell = InteractiveShellEmbed()
ipshell()
