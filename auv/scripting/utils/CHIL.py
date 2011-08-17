#! /usr/bin/env python
''' CAUV High-performance Information Log:

    Directory Structure:

    somelog.chil/
        a.log
        a.superlog
        b.log
        b.superlog
        megasuperlog


    log files:

    CHIL
    Format HG_FULL_REVISION_UID     # applies until next Format line is encountered
    Time YYYYMMDD-HHMMSS.SSSSSS     # Marks an absolute time
    TIMESTAMP MSG_ID(VALUE,VALUE,VALUE....,)  # time is integer (or long integer) musec since last recorded absolute time (Time line)


    !!! The format of the index files is still subject to change !!!

    superlog (index) files: seek positions are to nearest position *after* specified time
    
    CHILIDX
    Format HG_FULL_REVISION_UID         # applies until next Format line is encountered
    YYYYMMDD-HHMMSS.SSSSSS LONG_INTEGER # integer value is seek position in corresponding log file
    

    megasuperlog file:

    CHILSUPERIDX
    Format HG_FULL_REVISION_UID
    FILENAME ([MSG_ID YYYYMMDD-HHMMSS.SSSSSS--YYYYMMDD-HHMMSS.SSSSSS], ...) # any number of message id and time period sections

'''

# Standard Library
import os
import datetime
import time
import fcntl
import bisect
import functools
from base64 import b16encode, b16decode

# 3rd Party
import blist           # BSD license
import pyparsing as pp # MIT license

# CAUV
from hacks import sourceRevision
import cauv.messaging as messaging

class Const:
    Dat_Fname_Strftime_Fmt = '%Y%m%d-%H%M%S'
    Keyframe_Strftime_Fmt = '%Y%m%d-%H%M%S.%f'


# output structures for index grammars:
class MsgPeriod():
    def __init__(self, msgid, fr, to):
        self.msgid = msgid
        self.fr = fr
        self.to = to
    def __repr__(self):
        return '%s: %s to %s' % (self.msgid, self.fr, self.to)

# index and super index grammars:
p_hg_uuid   = pp.Word('0123456789abcdef')
p_timestamp = pp.Combine(pp.Word(pp.nums,exact=8) +
              pp.Literal('-') + pp.Word(pp.nums,exact=6) +
              pp.Literal('.') + pp.Word(pp.nums))
p_timestamp.setParseAction(lambda x: datetime.datetime.strptime(x[0], Const.Keyframe_Strftime_Fmt))
p_filename = pp.Word(pp.alphanums + '-_.')

p_idx_header_line = pp.Literal('CHILIDX')
p_format_line     = pp.Literal('Format') + p_hg_uuid;
p_format_line.setParseAction(lambda x: x[1])
p_seekpos         = pp.Combine(pp.Word(pp.nums) + pp.Optional(pp.Literal('L')))
p_seekpos.setParseAction(lambda x: long(x[0]))
p_seekpos_line    = p_timestamp + p_seekpos
p_idx_line        = p_idx_header_line ^\
                    p_format_line ^\
                    p_seekpos_line

p_superidx_header_line = pp.Literal('CHILSUPERIDX')
p_msgid         = pp.Word(pp.nums)
p_msgid.setParseAction(lambda x: int(x[0]))
p_msgid_period  = pp.Group(
                  pp.Suppress(pp.Literal('[')) +
                  p_msgid + p_timestamp + pp.Suppress(pp.Literal('--')) + p_timestamp +
                  pp.Suppress(pp.Literal(']')))
p_msgid_period.setParseAction(lambda x: MsgPeriod(*x[0]))
p_msgloc_line   = p_filename + pp.Suppress('(') + \
                  pp.delimitedList(p_msgid_period) + pp.Suppress(pp.Optional(',')) + \
                  pp.Suppress(')')
p_superidx_line = p_superidx_header_line ^\
                  p_format_line ^\
                  p_msgloc_line

def testMetaGrammars():
    print p_hg_uuid.parseString('6e01240480ec08eab327ad6a14b0179908b98dbb')
    print p_timestamp.parseString('20110817-200746.725440')
    print p_filename.parseString('some_stupid-filename.something')
    print p_format_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')
    print p_seekpos.parseString(str(12356789123123756176234L))
    print p_seekpos_line.parseString('20110817-200746.725440 1235142364123')
    print p_idx_line.parseString('20110817-200746.725440 1235142364123')
    print p_idx_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')
    print p_idx_line.parseString('CHILIDX')
    print p_msgid_period.parseString('[1234 19890203-123456.789012--20110817-201900.000000]')
    print p_msgloc_line.parseString('filename ([1234 19890203-123456.789012--20110817-201900.000000], [1234 19890203-123456.789012--20110817-201900.000000])')
    print p_superidx_line.parseString('filename ([1234 19890203-123456.789012--20110817-201900.000000], [1234 19890203-123456.789012--20110817-201900.000000],)')
    print p_superidx_line.parseString('Format 6e01240480ec08eab327ad6a14b0179908b98dbb')

def tddiv(l, r):
    l_musec = long(l.microseconds) + l.seconds*1000000L + l.days*1000000L*24L*3600L
    r_musec = long(r.microseconds) + r.seconds*1000000L + r.days*1000000L*24L*3600L
    ret = l_musec / float(r_musec)
    #print 'tddiv: %s / %s = %s / %s = %s' % (l, r, l_musec, r_musec, ret)
    return ret

def linearInterp(xlow, ylow, xhi, yhi, x):
    # TODO: handle xhi = xlo gracefully
    if x == xlow:
        r = ylow
    elif x == xhi:
        r = yhi
    else:
        n = ((yhi - ylow) * (x - xlow))
        d = (xhi - xlow)
        if type(n) == datetime.timedelta and type(d) == datetime.timedelta:
            # silly datetime.timedelta doesn't support division
            r = ylow + tddiv(n, d)
        else:
            r = ylow + n / d
    #print 'low: %s %s\n  x: %s\n hi::%s %s\n  -> %s' % (xlow, ylow, x, xhi, yhi, r)
    return r

# TODO next...
class ShelfConverter:
    pass

class LinearpiecewiseApprox(blist.sorteddict):
    def __init__(self, rfunc=round, interp=linearInterp):
        blist.sorteddict.__init__(self)
        self.interpolate = interp
        self.rfunc = rfunc
    def __getitem__(self, k):
        # interpolates if a value is not present for the specified key
        sorted_keys = self.keys()
        ilow = bisect.bisect(sorted_keys, k) - 1
        if ilow < 0 or (ilow == -1 and sorted_keys[0] != k):
            raise RuntimeError('out of range: lt')
        ihi = ilow + 1
        if ihi >= len(sorted_keys):
            if sorted_keys[ilow] != k:
                raise RuntimeError('out of range: gt')
            else:
                return sorted_keys[ilow]
        '''#dbg
        for i, y in enumerate(sorted_keys):
            if i == ilow:
                print i, y, '<-- low'
                print '=', k
            elif i == ihi:
                print i, y, '<-- hi'
            else:
                print i, y'''
        klow = sorted_keys[ilow]
        khi = sorted_keys[ihi]
        r = self.interpolate(
            klow, blist.sorteddict.__getitem__(self, klow),
            khi, blist.sorteddict.__getitem__(self, khi),
            k
        )
        return self.rfunc(r)

class CHILer:
    class Const:
        Dat_Extn = '.log'
        Idx_Extn = '.superlog'
        Dir_Extn = '.chil'
        Super_Index = 'megasuperlog'
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
        self.__megasuperlog = None
        print 'CHILer:%s:%s' % (self.dirname, self.source_revision)
    def __ensureDirectory(self, f):
        if not os.path.exists(f):
            os.makedirs(f)
    def lockAndOpenForA(self, fname):
        f = open(self.fileName(fname), 'a+')
        # on OS X, this blocks until lock is available
        fcntl.lockf(f, fcntl.LOCK_EX)
        return f
    def releaseAndClose(self, fobj):
        fobj.flush()
        fcntl.lockf(fobj, fcntl.LOCK_UN)
        fobj.close()
    def openForR(self, fname):
        f = open(self.fileName(fname), 'r')
        return f
    def getAndLockMegaSuperLog(self):
        if self.__megasuperlog is None:
            fn = self.Const.Super_Index
            f = self.lockAndOpenForA(fn)
            if self.isEmpty(fn):
                f.write('CHILSUPERIDX')
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
        return bool(os.stat(self.fileName(fname)).st_size)
    def timeFormat(self, t=None):
        if t is None:
            t = datetime.datetime.now()
        return t.strftime(Const.Keyframe_Strftime_Fmt)
    def tdToMusec(self, timedelta):
        r = 24*3600*1000000L*timedelta.days
        r += 60*1000000L*timedelta.seconds
        r += timedelta.microseconds
        return r
    def serialiseMessage(self, s):
        serialised =  s.chil()
        return serialised
    def baseNameFromSubName(self, subname):
        if subname is None:
            subname = datetime.datetime.now().strftime(Const.Dat_Fname_Strftime_Fmt) + self.Const.Dat_Extn
        if subname.endswith(self.Const.Dat_Extn):
            basename = subname[:-len(self.Const.Dat_Extn)]
        else:
            basename = subname
        return basename
    def deserialiseMessage(self, s):
        import utils.childecode.decode_61a58d327d9f229ef265be85c9bf2de03235814e as decode
        r = decode.p_Message.parseString(s)
        return r

class Logger(CHILer):
    def __init__(self, dirname, subname=None):
        CHILer.__init__(self, dirname)
        basename = self.baseNameFromSubName(subname)
        self.idxname = basename + self.Const.Idx_Extn
        self.datname = basename + self.Const.Dat_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, self.datname, self.idxname)
        self.datfile = self.lockAndOpenForA(self.datname)
        self.idxfile = self.lockAndOpenForA(self.idxname)
        self.last_keyframe_time = None
        self.last_absolute_time = None
        self.writeFormatLines()
        self.recorded_msg_types = {}

    def writeKeyframe(self, t=None):
        if t is None:
            t = datetime.datetime.now()
        l = '%s %s\n' % (self.timeFormat(t), self.datfile.tell())
        self.last_keyframe_time = t
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
        if self.last_absolute_time is None or\
           record_time < self.last_absolute_time or\
           record_time - self.last_absolute_time > self.Const.Dat_Time_Line_Freq:
            self.writeTimeLine(record_time)
        if self.last_keyframe_time is None or\
           record_time - self.last_keyframe_time > self.Const.Idx_Keyframe_Freq:
            self.writeKeyframe()
        musec_delta = self.tdToMusec(record_time - self.last_absolute_time)
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
        # TODO:
        # update superlog on close?
        self.updateMegaSuperLog()
        self.releaseAndClose(self.datfile)
        self.releaseAndClose(self.idxfile)
        self.datfile = None
        self.idxfile = None
        pass

    def updateMegaSuperLog(self):
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
            return Interval(other.lo, other.hi)
        lo = self.lo if self.lo < other.lo else other.lo
        hi = self.hi if other.hi < self.hi else other.hi
        return Interval(lo, hi)
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
        basename = self.baseNameFromSubName(subname)
        self.idxname = basename + self.Const.Idx_Extn
        self.datname = basename + self.Const.Dat_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, self.datname, self.idxname)
        self.datfile = self.openForR(self.datname)
        self.idxfile = self.openForR(self.idxname)
        self.__cursor = cursor
        # recorded_msg_types
        self.recorded_msg_types = {}
        # map of datetime -> seek value, linearly interpolates for values not
        # present
        self.seek_map = LinearpiecewiseApprox(round)
        for idxline in self.idxfile:
            print idxline
            # EDITING HERE
            # ... populate seek_map from index file
        self.idxfile.close()
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
        return self.nextMessageTime() < other.nextMessageTime()
    def __eq__(self, other):
        if not isinstance(other, ComponentPlayer):
            return NotImplemented
        if self.__cursor is not other.__cursor:
            raise RuntimeError('other does not share cursor')
        return self.nextMessageTime() == other.nextMessageTime()
    def absoluteTimeAtSeekPos(self, seekpos):
        # !!! TODO!
        # this will probably come from the index file?
        # (would require specification that all absolute time lines are
        # indexed)
        pass
    def lineAfterSeekPos(self, seekpos):
        # !!! TODO!
        pass
    def nextMessageTime(self, after_time=None):
        if after_time is None:
            after_time = self.cursor()
        seekpos = self.seek_map[after_time]
        line = self.lineAfterSeekPos(seekpos)
        if not line:
            return None
        tdiff   = datetime.timedelta(microseconds=int(line.split()[0]))
        abstime = self.absoluteTimeAtSeekPos(seekpos)
        return abstime
    def timeToNextMessage(self):
        abstime = self.nextMessageTime()
        return abstime - self.cursor()

    def timepct(self, percent, start_clip=None, end_clip=None):
        # Return the latest time at which 'percent' of the messages logged
        # between start_clip and end_clip had been sent.
        # If start (end) clip is omitted, the earliest (latest) time messages
        # were recorded is used.
        pass


class Player(CHILer):
    class PushCursor:
        def __init__(self, player, value, no_check=False):
            self.player = player
            self.value = value
            self.no_check = no_check
        def __enter__(self):
            self.player.pushCursor(value)
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
        with self.openForR(self.Const.Super_Index) as megasuperlog:
            for line in megasuperlog:
                try:
                    parsed = p_msgloc_line.parseString(line)
                except:
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
        self.components = blist.sortedlist(components_by_fname.itervalues())
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
    def timeToNextMessage(self):
        return self.components[0].timeToNextMessage()
    def msgDensity(self, start, stop, samples=10):
        # density = mean(1 / (time from sample time to next message in microsec))
        samples = []
        step = (stop - start) / samples
        with PushCursor(self, start, no_check=True):
            for i in xrange(0, samples):
                samples.append(1.0 / self.timeToNextMessage())
                self.advanceCursor(step)
        return mean(samples)
    




def testLogCoverage():
    import cauv.messaging as m
    l = Logger('test')
    l.log(m.SonarDataMessage(m.SonarDataLine([1,2,3,4,5],0,6400,50000,6400)))
    l.log(m.GraphableMessage('thingy', 4.78),
          record_time=datetime.datetime.now() - datetime.timedelta(milliseconds=50)
    )
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

if __name__ == '__main__': 
    testLogCoverage_start = datetime.datetime.now()
    testLogCoverage()
    testLogCoverage_end = datetime.datetime.now()
    testMetaGrammars()

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
    r = Player('test')
    
    N = 5
    densities = []
    for x in xrange(0,N):
        density = r.msgDensity(
            testLogCoverage_start + x*(1.0/N)*(testLogCoverage_end-testLogCoverage_start),
            testLogCoverage_start + (x+1)*(1.0/N)*(testLogCoverage_end-testLogCoverage_start)
        )
        densities.append(density)
        print density
    drawVHistogram(desnities, x_axis=(testLogCoverage_start, testLogCoverage_end))

    r.close()

