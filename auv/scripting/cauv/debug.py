import sys
import traceback
from messaging import debug, warning, error, info, setDebugName
_old_excepthook = sys.excepthook
def log_excepthook(etype, evalue, tb):
    _old_excepthook(etype, evalue, tb)
    error(''.join(traceback.format_exception(etype, evalue, tb)))
sys.excepthook = log_excepthook
