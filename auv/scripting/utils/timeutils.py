#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import time

class RelativeTimeCapability(object):
    def __init__(self):
        self.__tstart = time.time()
    def relativeTime(self):
        return time.time() - self.__tstart
