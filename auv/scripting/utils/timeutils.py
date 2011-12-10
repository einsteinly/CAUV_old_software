import time

class RelativeTimeCapability(object):
    def __init__(self):
        self.__tstart = time.time()
    def relativeTime(self):
        return time.time() - self.__tstart
