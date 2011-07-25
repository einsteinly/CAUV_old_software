#! /usr/bin/env python
''' CAUV High-performance Information Log:

Directory Structure:

somelog.chil/
    a.log
    a.superlog
    a.lock # lock file prevents more than one process writing to a.log and a.superlog at the same time
    b.log
    b.superlog
    megasuperlog


log files:

CHIL
Format HG_FULL_REVISION_UID # applies until next Format line is encountered
Time YYYYMMDD-HHMMSS.SSSSSS  # Marks an absolute time
TIMESTAMP MSG_ID(VALUE,VALUE,VALUE....,)  # time is integer (or long integer) musec since last recorded absolute time (Time line)


superlog (index) files: seek positions are to nearest position *after* specified time

CHILIDX
Format HG_FULL_REVISION_UID # applies until next Format line is encountered
YYYYMMDD-HHMMSS.SSSSSS LONG_INTEGER # integer value is seek position in corresponding log file
 

megasuperlog file:

CHILSUPERIDX
FILENAME ([MSG_ID YYYYMMDD-HHMMSS.SSSSSS--YYYYMMDD-HHMMSS.SSSSSS], ...) # any number of message id and time period sections

'''

import os
import datetime
import time
import fcntl

import blist

from hacks import sourceRevision


class ShelfConverter:
    pass


class CHILer:
    class Const:
        Dat_Extn = '.log'
        Idx_Extn = '.superlog'
        Dir_Extn = '.chil'
        Super_Index = 'megasuperlog'
        Dat_Fname_Strftime_Fmt = '%Y%m%d-%H%M%s'
        Keyframe_Strftime_Fmt = '%Y%m%d-%H%M%s'
        # Frequency to write keyframes to index files:
        Idx_Keyframe_Freq = datetime.timedelta(seconds=60)
        # Frequency of absolute time lines:
        Dat_Time_Line_Freq = datetime.timedelta(minutes=5)
    def __init__(self, dirname):
        if not dirname.endswith(self.Const.Dir_Extn):
            dirname = dirname.rstrip('/\\') + self.Const.Dir_Extn
        self.dirname = dirname
        self.source_revision = sourceRevision()
        self.__ensureDirectory(self.dirname)
    def __ensureDirectory(self, f):
        if not os.path.exists(f):
            os.makedirs(f)
    def lockAndOpenForA(self, fname):
        f = open(self.fileName(fname), 'a+')
        # lock the whole file, raises IOError errno=EACCES or EAGAIN on failure
        # (or it's supposed to, on OS X it seems to block....)
        fcntl.lockf(f, fcntl.LOCK_EX)
        return f
    def releaseAndClose(self, fobj):
        fobj.flush()
        fcntl.lockf(fobj, fcntl.LOCK_UN)
        fobj.close()
    def openForR(self, fname):
        f = open(self.fileName(fname), 'r')
    def fileName(self, fname):
        return os.path.join(self.dirname, fname)
    def isEmpty(self, fname):
        return bool(os.stat(self.fileName(fname)).st_size)
    def timeFormat(self, t=None):
        if t is None:
            t = datetime.datetime.now()
        return t.strftime(self.Const.Keyframe_Strftime_Fmt)
    def tdToMusec(self, timedelta):
        r = 24*3600*1000000L*timedelta.days
        r += 60*1000000L*timedelta.seconds
        r += timedelta.microseconds
        return r
    def serialiseField(self, f):
        ???
    def serialiseMsgFields(self, msg):
        def shouldSave(attr):
            if not k.startswith('__') and not k in ('group', 'msgId'):
                return True
            return False
            attrs = msg.__class__.__dict__
            r = []
            for k in filter(shouldSave, attrs):
                r.append(attrs[k].__get__(msg))
            return map(self.serialiseField, r)

class Logger(CHILer):
    def __init__(self, dirname, subname=None):
        CHILer.__init__(self, dirname)
        if subname is None:
            subname = datetime.datetime.now().strftime(self.Const.Dat_Fname_Strftime_Fmt) + self.Const.Dat_Extn
        if subname.endswith(self.Const.Dat_Extn):
            basename = subname[:-len(self.Const.Dat_Extn)]
        self.idxname = basename + self.Const.Idx_Extn
        self.datname = basename + self.Const.Dat_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, datname, idxname)
        self.datfile = self.lockAndOpenForA(self.datname)
        self.idxfile = self.lockAndOPenForA(self.idxname)
        self.last_keyframe_time = None
        self.last_absolute_time = None
        self.writeFormatLines()

    def writeKeyframe(self, t=None):
        l = '%s %s\n' % (self.timeFormat(t), self.datfile.tell())
        self.idxfile.write(l)

    def writeFormatLines(self):
        l = 'Format %s\n' % self.source_revision
        self.datfile.write(l)
        self.idxfile.write(l)
    
    def writeTimeLine(self, t=None):
        if t is None:
             t = datetime.datetime.now()
        self.last_absolute_time = t
        l = 'Time %s\n' % self.timeFormat(t)
        self.datfile.write(l)

    def log(self, msg, record_time=None):
        if record_time is None:
            record_time = datetime.datetime.now()
        if self.last_absolute_time is None or
           record_time < self.last_absolute_time or
           record_time - self.last_absolute_time > self.Const.Dat_Time_Line_Freq:
            self.writeTimeLine(record_time)
        if record_time - self.last_keyframe_time > self.Const.Idx_Keyframe_Freq:
            self.writeKeyframe()
        musec_delta = self.tdToMusec(record_time - self.last_absolute_time)
        msg_fields = self.serialiseMsgFields(msg)
        l = '%s %d(%s)\n' % (musec_delta, msg.msgId, ','.join(msg_fields))

    def close(self):
        # TODO:
        # update superlog
        # update the shared megasuperlog
        self.updateMegaSuperLog()
        self.releaseAndClose(self.datfile)
        self.releaseAndClose(self.idxfile)
        self.datfile = None
        self.idxfile = None
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
    r = Player('test')

