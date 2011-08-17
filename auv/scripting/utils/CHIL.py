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
Format HG_FULL_REVISION_UID # applies until next Format line is encountered
Time YYYYMMDD-HHMMSS.SSSSSS  # Marks an absolute time
TIMESTAMP MSG_ID(VALUE,VALUE,VALUE....,)  # time is integer (or long integer) musec since last recorded absolute time (Time line)


superlog (index) files: seek positions are to nearest position *after* specified time

CHILIDX
Format HG_FULL_REVISION_UID # applies until next Format line is encountered
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
from base64 import b16encode, b16decode

# 3rd Party
import blist           # BSD license
import pyparsing as pp # MIT license

# CAUV
from hacks import sourceRevision
import cauv.messaging as messaging

# TODO next...
class ShelfConverter:
    pass

def interestingAttrs(msg):
    # doesn't just work with messages
    def shouldSave(attr):
        if not attr.startswith('__') and not attr in ('group', 'msgId'):
            return True
        return False
    attrs = msg.__class__.__dict__
    r = []
    for k in filter(shouldSave, attrs):
        r.append(attrs[k].__get__(msg))
    return r

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
        return t.strftime(self.Const.Keyframe_Strftime_Fmt)
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
            subname = datetime.datetime.now().strftime(self.Const.Dat_Fname_Strftime_Fmt) + self.Const.Dat_Extn
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
        print l,
        print 'L:%s:=%s:=%s' % (msg.__class__.__name__, l[:-1], self.deserialiseMessage(serialised_msg))
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


class ComponentPlayer(CHILer):
    def __init__(self, dirname, subname):
        CHILer.__init__(self, dirname)
        basename = self.baseNameFromSubName(subname)
        self.idxname = basename + self.Const.Idx_Extn
        self.datname = basename + self.Const.Dat_Extn
        print '%s\n\t%s\n\t%s' % (self.dirname, self.datname, self.idxname)
        self.datfile = self.openForR(self.datname)
        self.idxfile = self.openForR(self.idxname)
        self.recorded_msg_types = {}
        # map of datetime -> seek value, linearly interpolates for values not
        # present
        self.seek_position = LinearpiecewiseApprox(round)

    def updateCachedIdx(self, megasuperlog_line):
        # interpret a line of the megasuperlog, update self.recorded_msg_types
        print megasuperlog_line
        pass

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
        #megasuperlogf = self. EDITING HERE
        
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
    l.log(m.AddArcMessage())
    l.close()

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

