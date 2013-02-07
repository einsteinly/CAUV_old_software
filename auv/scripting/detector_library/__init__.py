#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from glob import glob
from imp import find_module
path = find_module('detector_library')[1]
__all__ = [f[len(path)+1:-3] for f in glob(path+'/*.py') if f[-11:-3]!='__init__']
