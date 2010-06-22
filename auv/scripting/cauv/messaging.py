# This module transparently wrapped the boost-python exported messaging
# interafaace
import sys
_saved = sys.path
sys.path = [r'/home/jc593/Dev/hg-code/auv/scripting/bin/gcc-4.2.4/debug/'] + sys.path
from cauvinterface import *
sys.path = _saved

