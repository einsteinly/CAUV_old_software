#!/usr/bin/env python2.7
''' CAUV High-performance Information Log:

    Directory Structure:

    somelog.chil/
        a.log
        a.superlog
        b.log
        b.superlog
        megasuperlog


    # log files:

    CHIL
    Format HG_FULL_REVISION_UID               # applies until next Format line
                                              # is encountered
    Time YYYYMMDD-HHMMSS.SSSSSS               # Marks an absolute time EVERY
                                              # 'Time' line must be recorded in
                                              # the index file
    TIMESTAMP MSG_ID(VALUE,VALUE,VALUE....,)  # time is integer (or long
                                              # integer) musec since last
                                              # recorded absolute time ('Time'
                                              # line)

    # !!! The format of the index files is still subject to change !!!

    # superlog (index) files: seek positions are to nearest position *after*
    # specified time

    CHILIDX
    Format HG_FULL_REVISION_UID LONG_INT    # applies until next Format line
                                            # is encountered, LONG_INT
                                            # specifies the point in the log
                                            # file from which this format
                                            # should be used

    LONG_INTEGER YYYYMMDD-HHMMSS.SSSSSS     # integer is seek position in
                                            # corresponding log file. This form
                                            # is used to record Time lines
                                            # from the log file

    YYYYMMDD-HHMMSS.SSSSSS LONG_INTEGER     # integer value is seek position
                                            # in corresponding log file. This
                                            # line does not necessarily
                                            # correspond to a Time line in the
                                            # log file

    # megasuperlog file:
    # (any number of message id and time period sections are permitted)

    CHILSUPERIDX
    Format HG_FULL_REVISION_UID
    FILENAME ([MSG_ID YYYYMMDD-HHMMSS.SSSSSS--YYYYMMDD-HHMMSS.SSSSSS], ...)

'''

# Standard Library
import os
import datetime
import time
import fcntl
import functools
import sys
import copy
import math
import warnings
from base64 import b16encode, b16decode

# 3rd Party
import blist # BSD license
import thirdparty.pyparsing as pp # MIT license

# CAUV
from hacks import sourceRevision, tddiv, tdmul, tdToLongMuSec
from interpolation import LinearpiecewiseApprox, OutOfRange_Low, OutOfRange_High, zeroOrderInterp, linearInterp_timedeltas
import cauv.messaging as messaging


def debugAssert(cond):
    if not cond:
        import pdb
        pdb.set_trace()

class Const:
    Dat_Fname_Strftime_Fmt = '%Y%m%d-%H%M%S'
    Keyframe_Strftime_Fmt = '%Y%m%d-%H%M%S.%f'


# Index and super-index grammars
class IdxGrammar:
    class MsgPeriod():
        def __init__(self, msgid, fr, to):
            self.msgid = msgid
            self.fr = fr
            self.to = to
        def __repr__(self):
            return '%s: %s to %s' % (self.msgid, self.fr, self.to)
    p_hg_uuid   = pp.Word('0123456789abcdef')
    p_timestamp = pp.Combine(pp.Word(pp.nums,exact=8) +
                  pp.Literal('-') + pp.Word(pp.nums,exact=6) +
                  pp.Literal('.') + pp.Word(pp.nums))
    p_timestamp.setParseAction(lambda x: datetime.datetime.strptime(x[0], Const.Keyframe_Strftime_Fmt))
    p_filename = pp.Word(pp.alphanums + '-_.')

    p_idx_header_line = pp.Literal('CHILIDX') + pp.Suppress(pp.LineEnd())
    p_seekpos         = pp.Combine(pp.Word(pp.nums) + pp.Optional(pp.Literal('L')))
    p_seekpos.setParseAction(lambda x: long(x[0]))
    p_format_line     = pp.Suppress(pp.Literal('Format')) + p_hg_uuid + pp.Suppress(pp.LineEnd())
    p_idxformat_line  = pp.Suppress(pp.Literal('Format')) + p_hg_uuid + p_seekpos + pp.Suppress(pp.LineEnd())
    p_time_line       = pp.Suppress(pp.Literal('Time')) + p_timestamp + pp.Suppress(pp.LineEnd())
    p_timepos_line    = p_seekpos + p_timestamp + pp.Suppress(pp.LineEnd())
    p_seekpos_line    = p_timestamp + p_seekpos + pp.Suppress(pp.LineEnd())
    p_idx_line        = p_idx_header_line ^\
                        p_format_line ^\
                        p_seekpos_line

    p_superidx_header_line = pp.Literal('CHILSUPERIDX') + pp.Suppress(pp.LineEnd())
    p_msgid         = pp.Word(pp.nums)
    p_msgid.setParseAction(lambda x: int(x[0]))
    p_msgid_period  = pp.Group(
                      pp.Suppress(pp.Literal('[')) +
                      p_msgid + p_timestamp + pp.Suppress(pp.Literal('--')) + p_timestamp +
                      pp.Suppress(pp.Literal(']')))
    p_msgid_period.setParseAction(lambda x: IdxGrammar.MsgPeriod(*x[0]))
    p_msgloc_line   = p_filename + pp.Suppress('(') + \
                      pp.delimitedList(p_msgid_period) + pp.Suppress(pp.Optional(',')) + \
                      pp.Suppress(')') + pp.Suppress(pp.LineEnd())
    p_superidx_line = p_superidx_header_line ^\
                      p_format_line ^\
                      p_msgloc_line

''' TODO: this...
# deriving from the builtin file object is sort of not really possible...
class BetterFile:
    def __init__(self, actual_file):
        self.__f = actual_file
        self.linepos = '?'
    def __getattribute__(self, n):
        return self.__f.__getattribute__(n)
    def readPreviousLine(self):

    def readCurrentLine(self):
        # postcondition: tell() is at start of next line
        if self.linepos == 'start':
            return self.__f.readline()
        elif self.linepos == 'end':

        print 'readline...'
        self.__f.readline()
'''

class CHILer:
    class Const:
        Dat_Extn = '.log'
        Idx_Extn = '.superlog'
        Dir_Extn = '.chil'
        Super_Index = 'megasuperlog'
        # Frequency to write keyframes to index files:
        Idx_Keyframe_Freq = datetime.timedelta(seconds=60)
        # Maximum number of recorded messages between keyframes:
        Idx_Keyframe_nFreq = 1000
        # Frequency of absolute time lines:
        Dat_Time_Line_Freq = datetime.timedelta(minutes=5)
        Read_Buf_Size = 0x10000
    def __init__(self, dirname):
        if not dirname.endswith(self.Const.Dir_Extn):
            dirname = dirname.rstrip('/\\') + self.Const.Dir_Extn
        self.dirname = dirname
        self.source_revision = sourceRevision()
        self.__ensureDirectory(self.dirname)
        self.__megasuperlog = None
        print 'CHILer:%s:%s' % (self.dirname, self.source_revision)
    def __ensureDirectory(self, f):
        if not os.path.exists(f):
            os.makedirs(f)
    def lockAndOpenForA(self, fname):
        f = open(self.fileName(fname), 'a+b')
        # on OS X, this blocks until lock is available
        fcntl.lockf(f, fcntl.LOCK_EX)
        return f
    def releaseAndClose(self, fobj):
        fobj.flush()
        fcntl.lockf(fobj, fcntl.LOCK_UN)
        fobj.close()
    def openForR(self, fname):
        f = open(self.fileName(fname), 'rb')
        return f
    def getAndLockMegaSuperLog(self):
        if self.__megasuperlog is None:
            fn = self.Const.Super_Index
            f = self.lockAndOpenForA(fn)
            if self.isEmpty(fn):
                f.write('CHILSUPERIDX\n')
            self.__megasuperlog = f
            return f
        else:
            raise RuntimeError('log already open')
    def releaseAndCloseMegaSuperLog(self):
        if self.__megasuperlog is not None:
            self.releaseAndClose(self.__megasuperlog)
            self.__megasuperlog = None
        else:
            raise RuntimeError('no log to close')
    def fileName(self, fname):
        return os.path.join(self.dirname, fname)
    def isEmpty(self, fname):
        return bool(0 == os.stat(self.fileName(fname)).st_size)
    def timeFormat(self, t=None):
        if t is None:
            t = datetime.datetime.now()
        return t.strftime(Const.Keyframe_Strftime_Fmt)
    def serialiseMessage(self, s):
        return s.chil()
    @classmethod
    def dirAndSubNamesFromPath(cls, fname):
        dot_chil_idx = fname.find(cls.Const.Dir_Extn)
        subname = ''
        dirname = None
        if dot_chil_idx != -1:
            split_idx = dot_chil_idx + len(cls.Const.Dir_Extn)
            dirname = fname[:split_idx]
            # !!! TODO: proper path handling
            subname = fname[split_idx:].strip('./')
        if not len(subname):
            subname = None
        if not dirname:
            dirname = fname.rstrip('./') + '.chil'
        return (dirname, subname)
    def baseNameFromSubName(self, subname):
        if subname is None:
            subname = datetime.datetime.now().strftime(Const.Dat_Fname_Strftime_Fmt) + self.Const.Dat_Extn
        if subname.endswith(self.Const.Dat_Extn):
            basename = subname[:-len(self.Const.Dat_Extn)]
        else:
            basename = subname
        return basename

class Logger(CHILer):
    def __init__(self, dirname, subname=None):
        CHILer.__init__(self, dirname)
        self.basename = self.baseNameFromSubName(subname)
        self.idxname = self.basename + self.Const.Idx_Extn
        self.datname = self.basename + self.Const.Dat_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, self.datname, self.idxname)
        self.datfile = self.lockAndOpenForA(self.datname)
        self.idxfile = self.lockAndOpenForA(self.idxname)
        self.last_keyframe_time = None
        self.last_absolute_time = None
        self.last_log_time = None
        self.num_since_last_keyframe = 0
        self.__writeFormatLines()
        self.recorded_msg_types = {}
    def __writeKeyframe(self, t=None):
        # not public        
        if t is None:
            t = datetime.datetime.now()
        l = '%s %s\n' % (self.timeFormat(t), self.datfile.tell())
        self.last_keyframe_time = t
        self.idxfile.write(l)
    def __writeFormatLines(self):
        # not public        
        l_dat = 'Format %s\n' % self.source_revision
        self.datfile.write(l_dat)
        l_idx = 'Format %s %s\n' % (self.source_revision, self.datfile.tell())
        self.idxfile.write(l_idx)
    def __writeTimeLine(self, t=None):
        # not public
        if t is None:
             t = datetime.datetime.now()
        self.last_absolute_time = t
        l = self.timeFormat(t)
        #print 'Time', l
        self.idxfile.write('%s %s\n' % (self.datfile.tell()+1, l))
        self.datfile.write('Time %s\n' % l)
    def log(self, msg, record_time=None):
        # Log message msg at time record_time if record_time is omitted the
        # current time is used.
        if record_time is None:
            record_time = datetime.datetime.now()
        if self.last_log_time is not None and record_time < self.last_log_time:
            warnings.warn('Non-monotonically increasing record times are not supported!')
            self.__writeTimeLine(record_time)
            self.__writeKeyframe(record_time)
        if self.last_absolute_time is None or\
           record_time < self.last_absolute_time or\
           record_time - self.last_absolute_time > self.Const.Dat_Time_Line_Freq :
            self.__writeTimeLine(record_time)
        if self.last_keyframe_time is None or\
           record_time - self.last_keyframe_time > self.Const.Idx_Keyframe_Freq or\
           self.num_since_last_keyframe > self.Const.Idx_Keyframe_nFreq:
            self.__writeKeyframe(record_time)
            self.num_since_last_keyframe = 0
        else:
            self.num_since_last_keyframe += 1
        self.last_log_time = record_time
        musec_delta = tdToLongMuSec(record_time - self.last_absolute_time)
        serialised_msg = self.serialiseMessage(msg)
        l = '%s %s\n' % (musec_delta, serialised_msg)
        #print l,
        #print 'L:%s:=%s:=%s' % (msg.__class__.__name__, l[:-1], self.deserialiseMessage(serialised_msg))
        self.datfile.write(l)
        # update time range for which these messages are recorded
        t_range = self.recorded_msg_types.get(msg.msgId, (record_time, record_time))
        if record_time < t_range[0]:
            t_range = (record_time, t_range[1])
        elif record_time > t_range[1]:
            t_range = (t_range[0], record_time)
        self.recorded_msg_types[msg.msgId] = t_range
    def close(self):
        # this MUST be called when this Logger object will no longer be used
        # TODO: support contextlib
        self.__writeKeyframe(self.last_log_time)
        self.__writeTimeLine(self.last_log_time)
        self.__updateMegaSuperLog()
        self.releaseAndClose(self.datfile)
        self.releaseAndClose(self.idxfile)
        self.datfile = None
        self.idxfile = None
    def __updateMegaSuperLog(self):
        # lines of the form: FILENAME ([MSG_ID YYYYMMDD-HHMMSS.SSSSSS--YYYYMMDD-HHMMSS.SSSSSS], ...)
        msl = self.getAndLockMegaSuperLog()
        l = 'Format %s\n' % self.source_revision
        msl.write(l)
        # TODO: should record distinct time ranges for each message, rather
        # than just one range
        for k, v in self.recorded_msg_types.iteritems():
            msl.write('%s ([%d %s--%s],)\n' % (
                self.datname, k, self.timeFormat(v[0]), self.timeFormat(v[1])
            ))
        self.releaseAndCloseMegaSuperLog()


@functools.total_ordering
class ContinuousInterval:
    def __init__(self,lo=None,hi=None):
        self.lo = lo
        self.hi = hi
    def __or__(self, other):
        if not isinstance(other, ContinuousInterval):
            return NotImplemented
        if self.lo is None:
            assert(self.hi is None)
            return ContinuousInterval(other.lo, other.hi)
        lo = self.lo if self.lo < other.lo else other.lo
        hi = self.hi if other.hi < self.hi else other.hi
        return ContinuousInterval(lo, hi)
    def __lt__(self, other):
        if isinstance(other, ContinuousInterval):
            if self.lo is None:
                if other.lo is None:
                    return False
                return True
            elif other.lo is None:
                return True
            elif self.lo < other.lo:
                return True
            elif self.lo > other.lo:
                return False
            else:
                return self.hi < other.hi
        else:
            return self.lo < other
    def __eq__(self, other):
        if isinstance(other, ContinuousInterval):
            return self.lo == other.lo and self.hi == other.hi
        else:
            return NotImplemented
    def __repr__(self):
        return '[%s,%s]' % (self.lo, self.hi)

@functools.total_ordering
class ComponentPlayer(CHILer):
    def __init__(self, dirname, subname, cursor):
        # cursor should be a one-element list so that its value is shared
        # between different players
        # (actually, the list can have more than one element, those with
        #  indices higher than 0 being used to store a stack of previous
        #  cursors -- so only cursor[0] should ever be used anyway)
        CHILer.__init__(self, dirname)
        self.basename = self.baseNameFromSubName(subname)
        self.idxname = self.basename + self.Const.Idx_Extn
        self.datname = self.basename + self.Const.Dat_Extn
        print 'Scanning Component %s...' % self.basename
        #print '%s\n\t%s\n\t%s' % (self.dirname, self.datname, self.idxname)
        self.datfile = self.openForR(self.datname)
        self.__cursor = cursor
        # recorded_msg_types
        self.recorded_msg_types = {}
        # map of datetime -> seek value, linearly interpolates for values not
        # present
        self.seek_map = LinearpiecewiseApprox(round,interp=linearInterp_timedeltas)
        # at each indexed seek position the absolute time (used for relative
        # timestamps) must be recorded
        self.seek_time_map = LinearpiecewiseApprox(rfunc=lambda x:x, interp=zeroOrderInterp)
        self.decoders = LinearpiecewiseApprox(rfunc=lambda x:x, interp=zeroOrderInterp)
        self.default_decoder = None
        try:
            self.default_decoder = self.importDecoder(sourceRevision())
        except ImportError:
            print 'WARNING: no default decoder (current revision) available.'
            #import traceback
            #traceback.print_exc()
        else:
            print 'loaded default decoder (%s)' % sourceRevision()
        with self.openForR(self.idxname) as idxfile:
            for idxline in idxfile:
                try:
                    parsed = IdxGrammar.p_seekpos_line.parseString(idxline)
                    #print 'seekpos line:', parsed[0], parsed[1]
                    self.seek_map[parsed[0]] = parsed[1]
                    continue
                except pp.ParseException:
                    pass
                try:
                    parsed = IdxGrammar.p_timepos_line.parseString(idxline)
                    self.seek_time_map[parsed[0]] = parsed[1]
                    continue
                except pp.ParseException:
                    pass
                try:
                    parsed = IdxGrammar.p_idxformat_line.parseString(idxline)
                    try:
                        decoder = self.importDecoder(parsed[0])
                        self.decoders[parsed[1]] = decoder
                    except ImportError:
                        print 'No decoder for hg revision %s!' % parsed[0]
                        self.decoders[parsed[1]] = None
                        import traceback
                        traceback.print_exc()
                    else:
                        print 'loaded decoder for %s' % parsed[0]
                except pp.ParseException:
                    pass
        #print 'seek-time map: %s' % '\n\t'.join(map(str, sorted(self.seek_time_map.items())))
        #print '     seek map: %s' % '\n\t'.join(map(str, sorted(self.seek_map.items())))
        #print '     decoders: %s' % self.decoders
        print 'Scanned %s successfully.' % self.datname
    def importDecoder(self, name):
        import importlib
        for package  in ('utils.childecode', 'childecode'):
            modname = '%s.decode_%s' % (package,name)
            try:
                importlib.import_module(package)
                return importlib.import_module(modname, package)
            except ImportError:
                pass
            except AttributeError:
                pass
        raise
        
    def deserialise(self, msgstring, seekpos):
        decoder = self.decoders[seekpos]
        if decoder is None:
            decoder = self.default_decoder
        return decoder.parseMessage(msgstring)
    def swap(self, other):
        # dum di dum di dum
        t = other.__dict__
        other.__dict__ = self.__dict__
        self.__dict__ = t
    def close(self):
        self.datfile.close()
    def cursor(self):
        return self.__cursor[0]
    def updateCachedIdx(self, msg_period):
        # interpret a line of the megasuperlog, update self.recorded_msg_types
        if msg_period.msgid not in self.recorded_msg_types:
            self.recorded_msg_types[msg_period.msgid] = blist.sortedlist()
        self.recorded_msg_types[msg_period.msgid].add(ContinuousInterval(msg_period.fr,msg_period.to))
    def __lt__(self, other):
        if not isinstance(other, ComponentPlayer):
            return NotImplemented
        if self.__cursor is not other.__cursor:
            raise RuntimeError('other does not share cursor')
        s_nm = self.timeOfNextMessage()
        o_nm = other.timeOfNextMessage()
        if s_nm is None:
            return False
        elif o_nm is None:
            return True
        else:
            return s_nm < o_nm
    def __eq__(self, other):
        if not isinstance(other, ComponentPlayer):
            return NotImplemented
        if self.__cursor is not other.__cursor:
            raise RuntimeError('other does not share cursor')
        return self.timeOfNextMessage() == other.timeOfNextMessage()
    def absoluteTimeAtSeekPos(self):
        #print 'absoluteTimeAtSeekPos %d = %s' % (seekpos, self.seek_time_map[seekpos])
        return self.seek_time_map[self.datfile.tell()]
    def isMsgLine(self, line):
        if line.startswith('CHIL') or \
           line.startswith('Format') or \
           line.startswith('Time'):
            return False
        return True
    def timeOffsetOfMsgLine(self, line):
        if not self.isMsgLine(line):
            raise RuntimeError('not a message line')
        return datetime.timedelta(microseconds=long(line.split()[0]))
    def currentMsgLine_LeaveTellAtStart(self):
        # tell() can be anywhere, the line on which tell() currently is will be
        # returned
        # postcondition: tell() is at the start of the line following the 
        # returned line
        bufsize = self.Const.Read_Buf_Size
        lbegin = ''
        while True:
            try:
                self.datfile.seek(-bufsize, os.SEEK_CUR)
            except IOError:
                # at start of file
                bufsize = self.datfile.tell()
                self.datfile.seek(-bufsize, os.SEEK_CUR)
                buf = self.datfile.read(bufsize)
            else:
                buf = self.datfile.read(bufsize)
                # TODO: remove this check if it can never happen
                if len(buf) != bufsize:
                    print '?!', len(buf), bufsize, buf
                    break
            try:
                last_newline = buf.rindex('\n')
            except ValueError:
                self.datfile.seek(-bufsize, os.SEEK_CUR)
                lbegin = buf + lbegin
                continue
            else:
                lbegin = buf[last_newline+1:] + lbegin
                break
        line = lbegin + self.datfile.readline()
        while line and not self.isMsgLine(line):
            line = self.datfile.readline()
        #print 'currentMsgLine_LeaveTellAtStart %d:' % len(line), line
        return line
    def previousMsgLine_LeaveTellAtStart(self):
        # precondition: tell() at start of following line
        # postcondition: tell() at start of returned line
        bufsize = self.Const.Read_Buf_Size
        line = ''
        # move past previous newline
        try:
            self.datfile.seek(-1, os.SEEK_CUR)
        except IOError:
            # at start of file:
            return None
        while True:
            try:
                self.datfile.seek(-bufsize, os.SEEK_CUR)
            except IOError:
                # at start of file
                bufsize = self.datfile.tell()
                self.datfile.seek(-bufsize, os.SEEK_CUR)
                buf = self.datfile.read(bufsize)
            else:
                buf = self.datfile.read(bufsize)
                # TODO: remove this check if it can never happen
                if len(buf) != bufsize:
                    print '?!', len(buf), bufsize, buf
                    break
            try:
                last_newline = buf.rindex('\n')
            except ValueError:
                self.datfile.seek(-bufsize, os.SEEK_CUR)
                line = buf + line
                continue
            else:
                self.datfile.seek(1+last_newline - bufsize, os.SEEK_CUR)
                line = buf[last_newline+1:] + line
                break
        if not self.isMsgLine(line):
            return self.previousMsgLine_LeaveTellAtStart()
        else:
            #print 'previousMsgLine_LeaveTellAtStart %d:' % len(line), line
            return line
    def firstMsgLine_LeaveTellAtStart(self):
        # return first msg line of file
        self.datfile.seek(0)
        line = self.datfile.readline()
        while line and not self.isMsgLine(line):
            line = self.datfile.readline()
        return line
    def timeOfNextMessage(self):
        return self.nextMessageAndTime(deserialise=False)[1]
    def nextMessageAndDelta(self, deserialise=True):
        msg, time = self.nextMessageAndTime(deserialise)
        # optimisation: try the most common thing first, if it turns out that
        # time is None then we'll get a TypeError
        try:
            return (msg, time - self.__cursor[0])
        except TypeError:
            return None, None
    def nextMessageAndTime(self, deserialise=True):
        t = self.cursor()
        prev_line = None
        try:
            #print self.datname, 'cursor:', t
            seekpos = self.seek_map[t]
            #print self.datname, '->seek:', seekpos
        except OutOfRange_Low:
            #print self.datname, 'seek OutOfRange_Low', t
            line = self.firstMsgLine_LeaveTellAtStart()
            at_start = True
        except OutOfRange_High, e:
            #print self.datname, 'seek OutOfRange_High', t
            return None, None
        else:
            at_start = False
            self.datfile.seek(seekpos)
            line = self.currentMsgLine_LeaveTellAtStart()
        # Move backwards until we reach a line that's too early, or exactly
        # now, and then forwards until a line that's in the *future* (greater
        # than now)
        #        <--------S
        #        >|
        # or:
        #  <
        #  ------>|
        # now tell() is at the start of a line
        #print 'Gt:'
        while line and not at_start:
            #print self.timeOffsetOfMsgLine(line) + self.absoluteTimeAtSeekPos() - t,\
            #      self.timeOffsetOfMsgLine(line), self.absoluteTimeAtSeekPos(), self.datfile.tell()
            line = self.previousMsgLine_LeaveTellAtStart()
            try:
                if self.timeOffsetOfMsgLine(line) + self.absoluteTimeAtSeekPos() <= t:
                    #print self.timeOffsetOfMsgLine(line) + self.absoluteTimeAtSeekPos() - t,\
                    #      self.timeOffsetOfMsgLine(line), self.absoluteTimeAtSeekPos() , self.datfile.tell
                    at_start = True
                    # move to end of line this line
                    # !!!! I swear this +1 shouldn't be necessary!
                    self.datfile.seek(len(line)+1, os.SEEK_CUR)
            except IOError:
                import traceback
                traceback.print_exc()
                print 'line:', line
                print 'hit start of file!'
                at_start = True
                line = self.firstMsgLine_LeaveTellAtStart()
                break
        # now at tell() is at the start of the next line:
        #print 'Lt:'
        while line and self.timeOffsetOfMsgLine(line) + self.absoluteTimeAtSeekPos() <= t:
            #print self.timeOffsetOfMsgLine(line) + self.absoluteTimeAtSeekPos() - t,\
            #      self.timeOffsetOfMsgLine(line), self.absoluteTimeAtSeekPos(), self.datfile.tell()
            line = self.datfile.readline()
            while line and not self.isMsgLine(line):
                line = self.datfile.readline()
        if line:
            # then it's the first line with positive time delta
            tdiff = self.timeOffsetOfMsgLine(line)
            abstime = self.absoluteTimeAtSeekPos()
            msg_time = abstime + tdiff
            assert(tdToLongMuSec(tdiff) >= 0)
            assert(msg_time > t)
            #print 'Got:', abstime + tdiff - t, line
            msgstring = line[line.find(' '):]
            if deserialise:
                return self.deserialise(msgstring, self.datfile.tell()), msg_time
            else:
                return msgstring, msg_time
        else:
            return None, None
    def timeToNextMessage(self):
        msgtime = self.timeOfNextMessage()
        if msgtime is None:
            return None
        if msgtime < self.cursor():
            warnings.warn('-ve time to next message!')
            print 'msgtime:', msgtime
            print ' cursor:', self.cursor()
            print '   seek:', self.seek_map[self.cursor()]
            print 'abstime:', self.absoluteTimeAtSeekPos()
        return msgtime - self.cursor()

class Player(CHILer):
    class PushCursor:
        def __init__(self, player, value, no_check=False):
            self.player = player
            self.value = value
            self.no_check = no_check
        def __enter__(self):
            self.player.pushCursor(self.value)
            return self
        def __exit__(self, exc_type, exc_value, traceback):
            value = self.player.popCursor()
            if exc_type is None and value != self.value and not self.no_check:
                raise RuntimeError('cursor push/pop mismatch')
    def __init__(self, dirname):
        CHILer.__init__(self, dirname)
        self.__cursor = [datetime.datetime.now()]
        components_by_fname = {}
        # !!! TODO: should at least scan the directory and warn about files
        # that aren't in the index?
        # !!! TODO: should at least warn about the Format lines
        print 'Scanning %s...' % dirname
        with self.openForR(self.Const.Super_Index) as megasuperlog:
            for line in megasuperlog:
                try:
                    parsed = IdxGrammar.p_msgloc_line.parseString(line)
                except pp.ParseException:
                    continue
                fname = parsed[0]
                if not fname in components_by_fname:
                    components_by_fname[fname] = ComponentPlayer(dirname, fname, self.__cursor)
                component = components_by_fname[fname]
                for interval in parsed[1:]:
                    component.updateCachedIdx(interval)
        #for component in components_by_fname.values():
        #    print component.datname
        #    for k, v in component.recorded_msg_types.iteritems():
        #        print k, v
        print 'Ordering components...'
        self.components = blist.sortedlist(components_by_fname.itervalues())
        print 'Scanned %s indices successfully.' % dirname
    def close(self):
        for c in self.components:
            c.close()
    def wrapCursorMutatingOp(self, op):
        temp = tuple(self.components)
        # do whatever it is that breaks the sort order:
        op()
        # self.components now invalid, sort order has changed -- so update:
        self.components = blist.sortedlist(temp)
    def pushCursor(self, new_cursor):
        self.wrapCursorMutatingOp(functools.partial(self.__cursor.insert, 0, new_cursor))
    def popCursor(self):
        self.wrapCursorMutatingOp(functools.partial(self.__cursor.pop, 0))
    def advanceCursor(self, step):
        # felt like bind was a bit dangerous with iadd...
        def advCur():
            self.__cursor[0] += step
        self.wrapCursorMutatingOp(advCur)
    def setCursor(self, new_cursor):
        def setCur():
            self.__cursor[0] = new_cursor
        self.wrapCursorMutatingOp(setCur)
    def cursor(self):
        return self.__cursor[0]
    def nextMessage(self, deserialise=True):
        r, tdelta = self.components[0].nextMessageAndDelta(deserialise=deserialise)
        #print 'nextMessage:', r, tdelta
        # ---------------------------------------------------------------------
        # !!! TODO: scope for significant optimisation here:
        # For regions of time when messages are coming only from one component
        # we should just iterate over that component, and probably not even use
        # the index files, but just scan through lines until we reach the
        # (known) time of next component order swap.
        # Should also introduce an iterator interface for playing back messages
        # at any specified speed.
        # ---------------------------------------------------------------------
        if r is not None:
            assert(tdelta is not None)
            self.__cursor[0] += tdelta
            # NOW ORDERING OF self.components MAY BE BROKEN
            x = 0
            # could do this in log(n) instead of linearly, not sure if it'd
            # work out faster for practical cases...
            while x < len(self.components)-1:
                n_ttn = self.components[x+1].timeToNextMessage()
                x_ttn = self.components[x].timeToNextMessage()
                if n_ttn is not None and (x_ttn is None or n_ttn < x_ttn):
                    # sneaky
                    #print 'swap %d %d' % (x, x+1)
                    self.components[x].swap(self.components[x+1])
                    x += 1
                else:
                    break
            # now all is well
        return r, tdelta
    def timeToNextMessage(self):
        nextt = self.components[0].timeToNextMessage()
        # debug stuff:
        #for c in self.components[1:]:
        #    if nextt is None:
        #        assert(c.timeToNextMessage() is None)
        #    else:
        #        c_ttnm = c.timeToNextMessage()
        #        assert(c_ttnm is None or x_ttnm >= nextt)
        return nextt
    def timeOfNextMessage(self):
        return self.components[0].timeOfNextMessage()
    def msgDensity(self, start, stop, N=10):
        # density = mean(1 / (time from sample time to next message in microsec))
        samples = []
        step = (stop - start) / N
        #print 'msgDensity: push', start, 'step', step
        with self.PushCursor(self, start, no_check=True):
            for i in xrange(0, N):
                #print 'msgDensity: cursor is', self.cursor()
                ttn = self.timeToNextMessage()
                #print 'msgDensity: ttn', ttn
                if ttn is None:
                    continue
                if ttn > step:
                    samples.append(0)
                else:
                    assert(tdToLongMuSec(ttn) >= 0)
                    if tdToLongMuSec(ttn) < 100:
                        # hack hack
                        samples.append(0.01)
                    else:
                        samples.append(1.0 / tdToLongMuSec(ttn))
                self.advanceCursor(step)
        #print samples
        return sum(samples) / N


def drawVHistogram(densities,
                   width=60, height=10,
                   ch=':', halfch='.',
                   xaxis=None,
                   xaxis_print=str,
                   tick_bins=30):
    # This function draws things like this:
    #
    #     :              : :  :  :  . : ::::: : :  : :  :::  .       :   :
    #     :           .  :.:  : .::.::: ::::: : :  : :  :::  :       :   :
    #     :           : .:::: : ::::::: :::::::::::: :  :::.::  .    : : :
    #     : . .  .   :: ::::: ::::::::::::::::::::::::: ::::::. :  .:: : :.
    #     : : : ::  .:: ::::: ::::::::::::::::::::::::: ::::::: :  ::::: ::
    #     : : : ::: ::: ::::: :::::::::::::::::::::::::::::::::::  ::::: :::        :
    # ....::: :::::.:::::::::::::::::::::::::::::::::::::::::::::  :::::.:::  ..   .:
    # ::::::: :::::::::::::::::::::::::::::::::::::::::::::::::::: :::::::::. ::   ::
    # :::::::.::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: ::   ::.
    # :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::.:: . :::
    # |---------------------------------------'--------------------------------------|
    # 00:00:29.928703                  00:00:36.773747                        00:00:43.618790

    # 2011-08-20 23:20:37.022175    2011-08-20 23:20:38.584136   2011-08-20 23:20:40.146096

    if xaxis is None:
        xaxis = (0, len(densities))
    plot = []
    l = [' '] * width
    for x in xrange(0, height):
        plot.append(copy.copy(l))
    bins = [0] * width
    for i, d in enumerate(densities):
        # TODO: anti aliasing...
        idx = int(width*float(i)/len(densities))
        bins[idx] += d
    heights = []
    bmax = 1 if max(bins) == 0 else max(bins)
    for b in bins:
        heights.append(int(round(2*height*b/bmax)))
    for x, h in enumerate(heights):
        for y in xrange(0, int(h/2)):
            plot[y][x] = ch
        if h&1:
            assert(' ' == plot[h/2][x])
            plot[h/2][x] = halfch
    for l in reversed(plot):
        sys.stdout.write(''.join(l).rstrip() + '\n')
    l = ''
    r = ''
    label_length = len(xaxis_print(xaxis[0])+xaxis_print(xaxis[1]))/2
    for x in xrange(0, width):
        if x == 0:
            l += '|'
            r += xaxis_print(xaxis[0])
        elif x == width-1:
            l += '|'
            if len(r) < len(l):
                r += xaxis_print(xaxis[1])
        elif x % tick_bins == 0:
            l += "'"
            if len(r) < len(l) - label_length / 2:
                # eww special case:
                r += xaxis_print(xaxis[0] + tdmul((float(x)/width),(xaxis[1]-xaxis[0])))
        else:
            l += '-'
            if len(r) < len(l) - label_length / 2:
                r += ' '
    sys.stdout.write(l + '\n')
    sys.stdout.write(r + '\n')

    sys.stdout.flush()



# ---------------------------- Tests etc. ----------------------------

def testMetaGrammars():
    print IdxGrammar.p_hg_uuid.parseString('6e01240480ec08eab327ad6a14b0179908b98dbb')
    print IdxGrammar.p_timestamp.parseString('20110817-200746.725440')
    print IdxGrammar.p_filename.parseString('some_stupid-filename.something')
    print IdxGrammar.p_format_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')
    print IdxGrammar.p_idxformat_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb 123456')
    print IdxGrammar.p_seekpos.parseString(str(12356789123123756176234L))
    print IdxGrammar.p_seekpos_line.parseString('20110817-200746.725440 1235142364123')
    print IdxGrammar.p_timepos_line.parseString('1235142364123 20110817-200746.725440')
    print IdxGrammar.p_idx_line.parseString('20110817-200746.725440 1235142364123')
    print IdxGrammar.p_idx_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')
    print IdxGrammar.p_idx_line.parseString('CHILIDX')
    print IdxGrammar.p_msgid_period.parseString('[1234 19890203-123456.789012--20110817-201900.000000]')
    print IdxGrammar.p_msgloc_line.parseString('filename ([1234 19890203-123456.789012--20110817-201900.000000], [1234 19890203-123456.789012--20110817-201900.000000])')
    print IdxGrammar.p_superidx_line.parseString('filename ([1234 19890203-123456.789012--20110817-201900.000000], [1234 19890203-123456.789012--20110817-201900.000000],)')
    print IdxGrammar.p_superidx_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')

def testLogCoverage(loops=200):
    import cauv.messaging as m
    l = Logger('test')
    for x in xrange(0, loops):
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5],0,6400,50000,6400)))
        # not supported (and probably won't be)
        #l.log(m.GraphableMessage('thingy', 4.78),
        #      record_time=datetime.datetime.now() - datetime.timedelta(milliseconds=50)
        #)
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5],0,6400,50000,6400)))
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5,6,7,8,9],1,6400,50000,6400)))
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5,4,3,2,1],2,6400,50000,6400)))
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5,5,5],3,6400,50000,6400)))
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5,0,2,4],4,6400,50000,6400)))
        l.log(m.SonarDataMessage(m.SonarDataLine([],5,6400,50000,6400)))

        # every message:
        l.log(m.MembershipChangedMessage('some group'))
        l.log(m.DebugMessage(m.DebugType.Error, 'an error message'))
        l.log(m.DebugMessage(m.DebugType.Error, '')) # zero-length string!
        l.log(m.DebugLevelMessage(5));
        l.log(m.MotorMessage(m.MotorID.Prop, 127));
        l.log(m.BearingAutopilotEnabledMessage(True, 0.123456789))
        l.log(m.BearingAutopilotParamsMessage(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9))
        l.log(m.DepthAutopilotEnabledMessage(True, 0.9876543210))
        l.log(m.DepthAutopilotParamsMessage(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9))
        l.log(m.DepthCalibrationMessage(0.11111111111111,0.222222222222222,0.33333333333333,0.444444444444444))
        l.log(m.PitchAutopilotEnabledMessage(True, 8.7654321098))
        l.log(m.PitchAutopilotParamsMessage(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9))
        l.log(m.StateRequestMessage())
        l.log(m.ScriptMessage(m.ScriptExecRequest('some script', 0.987654321, 'some id', 0x89012345)))
        l.log(m.MotorRampRateMessage(-123456, -234567))
        l.log(m.SetMotorMapMessage(m.MotorID.VBow, m.MotorMap(-1,-2,3,4)))
        l.log(m.ResetMCBMessage())
        l.log(m.CalibrateNoRotationMessage(0x8001))
        l.log(m.StateMessage(m.floatYPR(0.2,-4.542e5,+23e-23)))
        l.log(m.TelemetryMessage(m.floatYPR(-23.5e6, -21,-0), -123.456))
        l.log(m.BatteryUseMessage(12.34,56.78,-0.000001))
        l.log(m.ProcessStatusMessage('\0some process', 'status\r', 100.0, 1.0e2, 0xffffffff))
        l.log(m.LocationMessage(1.234567890123456789, 2.345678901234567890, 3.456789012345678901, m.floatXYZ(0,0,0)))
        l.log(m.GPSLocationMessage(0.1,0.2,0.3,0.4,0.5,0.6))
        l.log(m.SonarLocationMessage(m.floatXY(-1,2)))
        # !!! TODO: images?
        #l.log(m.ImageMessage(m.CameraID.Forward,None,m.TimeStamp(0x7fffffff,-0x8000000)))
        l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5,5,5],3,6400,50000,6400)))
        l.log(m.SonarControlMessage(0x7fff, 0x8000,0xff,0xffffffff,0xfffabcd,0xbc))
        parents = m.NodeInputArcVec()
        parents.append(m.NodeInputArc('some input', m.NodeOutput(12,'some output',m.OutputType.Image)))
        parents.append(m.NodeInputArc('some other', m.NodeOutput(12,'some other output',m.OutputType.Parameter)))
        children = m.NodeOutputArcVec();
        children.append(m.NodeOutputArc(m.NodeInput(13, 'some input'),'some output'))
        children.append(m.NodeOutputArc(m.NodeInput(13, 'some other input'),'some other output'))
        children.append(m.NodeOutputArc(m.NodeInput(13, 'some third input'),'some third output'))
        l.log(m.AddNodeMessage('pipeline name', m.NodeType.FileInput, parents, children))
        l.log(m.RemoveNodeMessage('nnnnn', 12345))
        l.log(m.ClearPipelineMessage('mm'))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'int param', m.NodeParamValue.create(123)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'float param', m.NodeParamValue.create(1.23)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'string param', m.NodeParamValue.create('test')))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'bool param', m.NodeParamValue.create(False)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Corner> param', m.NodeParamValue.create(m.CornerVec())))
        corners = m.CornerVec(); corners.append(m.Corner())
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Corner> param', m.NodeParamValue.create(corners)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Line> param', m.NodeParamValue.create(m.LineVec())))
        lines = m.LineVec(); lines.append(m.Line()); lines.append(m.Line())
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Line> param', m.NodeParamValue.create(lines)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Circle> param', m.NodeParamValue.create(m.CircleVec())))
        circles = m.CircleVec(); circles.append(m.Circle())
        l.log(m.SetNodeParameterMessage('some pl', 3, 'list<Circle> param', m.NodeParamValue.create(circles)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'a param', m.NodeParamValue.create(m.floatVec())))
        floats = m.floatVec(); floats.append(1); floats.append(2)
        l.log(m.SetNodeParameterMessage('some pl', 3, 'a param', m.NodeParamValue.create(floats)))
        l.log(m.SetNodeParameterMessage('some pl', 3, 'a param', m.NodeParamValue.create(m.KeyPointVec())))
        kps = m.KeyPointVec(); kps.append(m.KeyPoint())
        l.log(m.SetNodeParameterMessage('some pl', 3, 'a param', m.NodeParamValue.create(kps)))
        l.log(m.AddArcMessage('', m.NodeOutput(12,'',m.OutputType.Image), m.NodeInput(13, '')))
        l.log(m.RemoveArcMessage('', m.NodeOutput(12,'',m.OutputType.Image), m.NodeInput(13, '')))
        import struct
        l.log(m.GraphRequestMessage(struct.pack('<'+20*'B',*range(0,20)))) # ;)
        l.log(m.ForceExecRequestMessage('', 123))
        l.log(m.PipelineDiscoveryRequestMessage())
        l.log(m.PipelineDiscoveryResponseMessage('moo'))
        linevec = m.LineVec(); linevec.append(m.Line(m.floatXYZ(0,0,0), 3.1416, 3))
        l.log(m.LinesMessage('some lines', linevec))
        circlevec = m.CircleVec(); circlevec.append(m.Circle(m.floatXYZ(0,0,0), 3))
        l.log(m.CirclesMessage('some circles', circlevec))
        cornervec = m.CornerVec(); cornervec.append(m.Corner(m.floatXYZ(0,0,0), 3, 4, 5))
        l.log(m.CornersMessage('some corners', cornervec))
        kpvec = m.KeyPointVec(); kpvec.append(m.KeyPoint(m.floatXY(1,2), 1.0, 2, 3.0, 4, 5.0))
        l.log(m.KeyPointsMessage('some kps', kpvec))
        l.log(m.HistogramMessage('name', m.floatVec()))
        floatvec = m.floatVec(); floatvec.append(1); floatvec.append(2); floatvec.append(3);
        l.log(m.HistogramMessage('name', floatvec))
        l.log(m.CentreMessage('name', 0.1, 0.2))
        l.log(m.ControllerStateMessage(m.Controller.ManualOverride,0.1,0.2,0.3,0.4,0.5,0.6,0.7,m.MotorDemand(0,0,0,0,0)))
        # I give up...
        l.log(m.MotorStateMessage())
        l.log(m.ScriptResponseMessage())
        l.log(m.GraphableMessage())
        l.log(m.NodeAddedMessage())
        l.log(m.NodeRemovedMessage())
        l.log(m.NodeParametersMessage())
        l.log(m.GraphDescriptionMessage())
        l.log(m.ArcAddedMessage())
        l.log(m.ArcRemovedMessage())
        l.log(m.StatusMessage())
        l.log(m.InputStatusMessage())
        l.log(m.OutputStatusMessage())
        #l.log(m.GuiImageMessage())
        l.log(m.AliveMessage())
        l.log(m.PressureMessage())
        l.log(m.AIMessage())
        l.log(m.AIlogMessage())
        l.log(m.LightMessage())
        l.log(m.CuttingDeviceMessage())
        l.log(m.BatteryStatusMessage())
    l.close()

def testLinearPWA():
    p = LinearpiecewiseApprox(round)
    s = datetime.datetime.now()
    p[s] = 3
    time.sleep(0.1)
    p[datetime.datetime.now()] = 4
    p[datetime.datetime.now()] = 5
    time.sleep(0.1)
    e = datetime.datetime.now()
    p[e] = 6
    while s <= e:
        print '%s = %s' % (s, p[s])
        s += datetime.timedelta(milliseconds=25)
    print '%s = %s' % (s, p[e])

def testDensityMap(r, start_t, end_t):
    N = 80
    densities = []
    for x in xrange(0,N):
        density = r.msgDensity(
            start_t + tdmul(x*(1.0/N),(end_t-start_t)),
            start_t + tdmul((x+1)*(1.0/N),(end_t-start_t)),
            N=3
        )
        densities.append(density)
        #print 'density:', density

    def pdtshort(dt):
        return dt.strftime('%H:%H:%S.%f')
    drawVHistogram(
        densities, xaxis=(start_t, end_t),
        tick_bins=40, width=N, xaxis_print=pdtshort
    )

def testPlayback(r, start_t, end_t):
    r.setCursor(start_t)
    s = 0
    n = 0
    while r.cursor() < end_t:
        m, td = r.nextMessage()
        if m is None:
            break
        s += len(str(m))
        n += 1
        if n & 0x3ff == 0:
            print '%d messages, %d kB, %s' % (n, s/1000.0, r.cursor())
    print '%d messages, %d kB' % (n, s/1000.0)


if __name__ == '__main__':
    import cProfile
    testMetaGrammars()
    testLinearPWA()

    t_start = datetime.datetime.now()
    #cProfile.run('testLogCoverage()', 'chil_log.profile')
    t_end = datetime.datetime.now()

    t_start = datetime.datetime.strptime('20110824-190925.903656', Const.Keyframe_Strftime_Fmt)
    t_end = datetime.datetime.strptime('20110824-190927.724978', Const.Keyframe_Strftime_Fmt)

    r = Player('test')
    #cProfile.run('r.msgDensity(t_start, t_end, 65)', 'chil_msgDensity.profile')
    cProfile.run('testDensityMap(r, t_start, t_end)', 'chil_msgDensity.profile')
    cProfile.run('testPlayback(r, t_start, t_end)', 'chil_playMessages.profile')
    r.close()

    import pstats
    #logstats = pstats.Stats('chil_log.profile').strip_dirs()
    #logstats.sort_stats('time').print_stats(20)
    #logstats.print_callers(0.3)

    #densitystats = pstats.Stats('chil_msgDensity.profile').strip_dirs()
    #densitystats.sort_stats('time').print_stats(20)

    playstats = pstats.Stats('chil_playMessages.profile').strip_dirs()
    playstats.sort_stats('time').print_stats()
    #playstats.sort_stats('cumulative').print_stats()
    #playstats.print_callers(0.2)
