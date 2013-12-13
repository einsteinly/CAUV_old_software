import sys
import traceback

from rospy import logdebug as debug
from rospy import logerr as error
from rospy import logwarn as warning
from rospy import logout as info

_old_excepthook = sys.excepthook
def log_excepthook(etype, evalue, tb):
    _old_excepthook(etype, evalue, tb)
    error(''.join(traceback.format_exception(etype, evalue, tb)))
sys.excepthook = log_excepthook
