''' CAUV High-performance Information Log:

Directory Structure:

somelog.chil/
    a.log
    a.superlog
    a.lock # lock file prevents more than one process writing to a.log and a.superlog at the same time
    b.log
    b.superlog
    megasuperlog


childat files: plain text, whitespace in lines ignored, newlines significant

CHIL 
Format ${hg tip} 
Time YYYY-MM-DD-HH-MM-SS.SSSSSS  # Marks an absolute time
Data
TIMESTAMP MSG_ID(VALUE,VALUE,VALUE....,)  # time is integer (or long integer) musec since 

'''

import os
import datetime
import time

import blist

from hacks import sourceRevision


class ShelfConverter:
    pass


class CHILer:
    class Const:
        Dat_Extn = '.log'
        Idx_Extn = '.superlog'
        Dir_Extn = '.chil'
        Lock_Extn = '.lock'
        Super_Index = 'megasuperlog'
        Dat_Strftime_Fmt = '%Y%m%d-%H%M%s'
    def __init__(self, dirname):
        if not dirname.endswith(self.Const.Dir_Extn):
            dirname = dirname.rstrip('/\\') + self.Const.Dir_Extn
        self.dirname = dirname
        self.source_revision = sourceRevision()
        self.__ensureDirectory(self.dirname)
    def __ensureDirectory(self, f):
        if not os.path.exists(f):
            os.makedirs(f)
   

class Logger(CHILer):
    def __init__(self, dirname, subname=None):
        CHILer.__init__(self, dirname)
        if subname is None:
            subname = datetime.datetime.now().strftime(self.Const.Dat_Strftime_Fmt) + self.Const.Dat_Extn
        if subname.endswith(self.Const.Dat_Extn):
            basename = subname[:-len(self.Const.Dat_Extn)]
        idxname = basename + self.Const.Idx_Extn
        datname = basename + self.Const.Dat_Extn
        lockname = basename + self.Const.Lock_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, datname, idxname)
    
    def log(self, msg, record_time=None):
        if record_time is None:
            record_time = datetime.datetime.now()
        

    def close(self):
        # TODO:
        # update superlog
        # remove lock file
        # update the shared megasuperlog
        pass

class ComponentPlayer(CHILer):
    def __init__(self, dirname):
        CHILer.__init__(self, dirname)

    def timepct(self, percent, start_clip=None, end_clip=None):
        # Return the latest time at which 'percent' of the messages logged
        # between start_clip and end_clip had been sent.
        # If start (end) clip is omitted, the earliest (latest) time messages
        # were recorded is used.
        pass

class Player(CHILer):
    def __init__(self, dirname):
        CHILer.__init__(self, dirname)
        self.components = blist.sortedlist()
        
    def timepct(self, percent, start_clip=None, end_clip=None):
        # Return the latest time at which 'percent' of the messages logged
        # between start_clip and end_clip had been sent.
        # If start (end) clip is omitted, the earliest (latest) time messages
        # were recorded is used.
        pass


if __name__ == '__main__':
    import cauv.messaging as m
    l = Logger('test')
    l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5],0,6400,50000,6400)))
    l.log(m.GraphableMessage('thingy', 4.78),
          record_time=datetime.datetime.now() - datetime.timedelta(milliseconds=50)
    )
    l.close()
    r = Reader('test')

