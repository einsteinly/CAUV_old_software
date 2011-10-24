#
# Base class for message loggers
# The shelf format is deprecated, use CHIL instead (utils/CHIL.py).
#

# Standard Library
import shelve
import datetime
import sys
import math
import time
import cmd
import threading
import bisect
import traceback

# CAUV
import cauv.messaging as msg
import cauv.node as node
from cauv.debug import debug, info, warning, error
from utils import CHIL
from utils.hacks import tdToFloatSeconds


#TODO: there's quite a lot of code duplication here, we should add some of this
# to utils/

def floatToDatetime(ftime):
    return datetime.datetime(1970,1,1,0,0,0,0) + datetime.timedelta(seconds=ftime)

Datetime_Format = '%a %d %b %Y %H:%M:%S'

Ignore_Message_Attrs = (
    'group',
    'chil'
)

# These three classes need to be kept around so we can load the saved data from
# 2011-05-28
class YPRWrapper:
    def __init__(self, fypr = msg.floatYPR()):
        self.yaw = fypr.yaw
        self.pitch = fypr.pitch
        self.roll = fypr.roll
    def floatYPR(self):
        return msg.floatYPR(self.yaw, self.pitch, self.roll)

class SonarDataLineWrapper:
    def __init__(self, sdl = msg.SonarDataLine()):
        self.data = tuple(sdl.data)
        self.bearing = sdl.bearing
        self.bearingRange = sdl.bearingRange
        self.range = sdl.range
        self.scanWidth = sdl.scanWidth
    def SonarDataLine(self):
        dl = msg.byteVec()
        for b in self.data:
            dl.append(b)
        # default value for new field!
        if not hasattr(self, 'scanWidth'):
            self.scanWidth = 6400
        return msg.SonarDataLine(dl, self.bearing, self.bearingRange, self.range, self.scanWidth)
 
class MotorIDWrapper:
    def __init__(self, mid = msg.MotorID()):
        self.value = int(mid)
    def MotorID(self):
        return msg.MotorID(self.value)

def dictFromMessage(message):
    attrs = message.__class__.__dict__
    r = {}
    for k in attrs:
        if not k.startswith('__') and not k in Ignore_Message_Attrs:
            # hack hack hack hack hack
            # I'm sure there is a simpler way of doing this, but m.__dict__ is
            # unhelpfully empty...
            r[k] = attrs[k].__get__(message)
    r['__message_name__'] = message.__class__.__name__
    return r

def dictToMessage(attr_dict):
    new_attrs = {}
    for k in attr_dict:
        if k.startswith('__'):
            continue
        if attr_dict[k].__class__ == SonarDataLineWrapper:
            new_attrs[k] = attr_dict[k].SonarDataLine()
        elif attr_dict[k].__class__ == YPRWrapper:
            new_attrs[k] = attr_dict[k].floatYPR()
        elif attr_dict[k].__class__ == MotorIDWrapper:
            new_attrs[k] = attr_dict[k].MotorID()
        else:
            new_attrs[k] = attr_dict[k]
    return getattr(msg, attr_dict['__message_name__'])(**new_attrs)

def incFloat(f):
    if f == 0.0:
        return sys.float_info.min
    m, e = math.frexp(f)
    return math.ldexp(m + sys.float_info.epsilon / 2, e)


class MessageLoggerComment:
    def __init__(self, comment_str):
        self.datetime = datetime.datetime.now()
        self.comment = comment_str

    def __repr__(self):
        return 'Comment: %s (recorded at %s)' % (
            self.comment, self.datetime.strftime(Datetime_Format)
        )


class MessageLoggerSession:
    def __init__(self, info_str):
        self.datetime = datetime.datetime.now()
        self.info = info_str

    def __repr__(self):
        return 'New session (%s), recorded at %s' % (
            self.info, self.datetime.strftime(Datetime_Format)
        )

class LoggerBase(msg.MessageObserver):
    def __init__(self, cauv_node, do_record, playback_rate = 1.0):
        msg.MessageObserver.__init__(self)
        self.node = cauv_node
        self.tzero = time.time()
        self.profile_playback = False
        self.__recording = False
        self.__playback_lock = threading.Lock()
        self.__playback_active = False
        self.__playback_finished = threading.Condition()
        self.__playback_thread = None
        self.__playback_rate = playback_rate
        self.__playback_start_time = 0
    def relativeTime(self):
        return time.time() - self.tzero
    def playbackIsActive(self):
        self.__playback_lock.acquire()
        r =  self.__playback_active
        self.__playback_lock.release()
    def setPlaybackRate(self, rate):
        # messages are played back at rate * real time (higher is faster)
        self.__playback_rate = rate
    def playbackRate(self):
        return self.__playback_rate
    def startPlayback(self, start_time):
        if self.__playback_active:
            error("can't start playback: playback is already active")
            return
        self.__playback_lock.acquire()
        self.__playback_start_time = start_time
        self.__playback_active = True
        self.__playback_lock.release() 
        if self.profile_playback:
            self.__playback_thread = threading.Thread(target=self.profilePlayback)
        else:
            self.__playback_thread = threading.Thread(target=self.playbackRunloop)
        self.__playback_thread.start()
    def playbackStartTime(self):
        # currently relative to the start of the particular message log
        return self.__playback_start_time
    def profilePlayback(self):
        import profilehooks
        profilehooks.profile(self.playbackRunloop,filename='messageLogger-playback.profile', )()
    def playbackRunloop(self):
        raise NotImplementedError()
    def stopPlayback(self):
        if not self.__playback_active:
            return
        self.__playback_lock.acquire()
        self.__playback_active = False
        self.__playback_lock.release()
        self.__playback_finished.acquire()
        debug('waiting for playback thread...')
        self.__playback_finished.wait()
        self.__playback_finished.release()
        self.__playback_thead = None
    def playbackHasBeenStopped(self):
        r = False
        self.__playback_lock.acquire()
        if not self.__playback_active:
            r = True
        self.__playback_lock.release()
        return r
    def playbackDidFinish(self):
        self.__playback_finished.acquire()
        self.__playback_finished.notify()
        self.__playback_finished.release()
        self.__playback_lock.acquire()
        self.__playback_active = False
        self.__playback_lock.release()
    def doRecord(self, do_record):
        if self.__recording == do_record:
            return
        self.__recording = do_record
        if do_record:
            if self.playbackIsActive():
                self.stopPlayback()
            self.node.addObserver(self)
            debug('recording started')            
        else:
            self.node.removeObserver(self)
            debug('recording stopped')
    def recordingIsActive(self):
        return self.__recording

class CHILLogger(LoggerBase):
    def __init__(self, cauv_node, log_fname, do_record, playback_rate = 1.0):
        LoggerBase.__init__(self, cauv_node, do_record, playback_rate)
        self.__logger = None
        self.dirname = None
        self.subname = None
        self.setFile(log_fname)
        self.doRecord(do_record)
    def close(self):
        self.doRecord(False)
        if self.playbackIsActive():
            self.stopPlayback()
        if self.__logger:
            self.__logger.close()
        self.__logger = None
        self.__player = None
    def convertToCHIL(self, dname, subname=None):
        info('log is already in CHIL format') 
    def playbackRunloop(self):
        debug('playback started')
        p = CHIL.Player(self.dirname) 
        # find time of first message:
        p.setCursor(datetime.datetime(year=datetime.MINYEAR, month=1, day=1))
        tzero = p.timeOfNextMessage()
        p.setCursor(tzero + datetime.timedelta(seconds=self.playbackStartTime()))
        # find the time of the last message:
        # ... not implemented yet (wouldn't be too difficult)
        # tend = p.timeOfLastMessage()
        tstart_playback = self.relativeTime()
        try:
            while not self.playbackHasBeenStopped():
                m, td = p.nextMessage()
                if m is None:
                    break
                time_to_sleep_for = tdToFloatSeconds(td) - self.playbackRate() * (self.relativeTime() - tstart_playback)
                debug('sleeping for %gs' % time_to_sleep_for, 5)
                if time_to_sleep_for/self.playbackRate() > 10:
                    warning('more than 10 seconds until next message will be sent (%gs)' %
                            (time_to_sleep_for/self.playbackRate()))
                while time_to_sleep_for > 0 and not self.playbackHasBeenStopped():
                    playback_rate = self.playbackRate()
                    sleep_step = min((time_to_sleep_for/playback_rate, 0.2))
                    time_to_sleep_for -= sleep_step*playback_rate
                    time.sleep(sleep_step)
                #print 's: %s' % m.__class__.__name__
                sys.stdout.write('.'); sys.stdout.flush()
                self.node.send(m)
        except Exception, e:
            error('error in playback: ' + str(e))
            raise
        finally:
            self.playbackDidFinish()
        debug('playback finished')
    def setFile(self, dirname, subname=None):
        if subname is None:
            dirname, subname = CHIL.CHILer.dirAndSubNamesFromPath(dirname)
        if self.__logger is not None:
            if self.__logger.dirname != dirname and \
               self.__logger.basename != self.__logger.baseNameFromSubName(subname):
                self.__logger.close()
            else:
                # else set to the same thing, nothing to do
                return
        self.dirname = dirname
        self.subname = subname
        self.__logger = CHIL.Logger(dirname, subname)
        self.onNewSession()
    def onNewSession(self):
        # derived classes can override this
        pass
    def addComment(self, comment_str):
        info('recoding comment "%s"' % comment_str)
        self.logMessage(msg.DebugMessage(msg.DebugType.Info, comment_str))
    def logObject(self, d):
        warning('CHIL does not support generic object logging')
        self.logMessage(msg.DebugMessage(msg.DebugType.Debug, str(d)))
    def logMessage(self, m):
        if self.recordingIsActive():        
            self.__logger.log(m)

class _DeprecatedShelfLogger(LoggerBase):
    def __init__(self, cauv_node, shelf_fname, do_record, playback_rate = 1.0):
        LoggerBase.__init__(self, cauv_node, do_record, playback_rate)
        self.__shelf = None
        self.__cached_keys = set()
        self.__shelf_fname = None # set by setFile
        self.setFile(shelf_fname)
        self.doRecord(do_record)
        warning('This logging class is deprecated, use CHILLogger instead')

    def setFile(self, fname):
        if fname.endswith('chil'):
            raise RuntimeError('CHIL files must be opened with a CHIL logger')
        if self.__shelf is not None and self.__shelf_fname != fname:
            self.stopPlayback()
            old_shelf = self.__shelf()
            self.__shelf = shelve.open(fname)
            old_shelf.close()
        elif self.__shelf is None:
            self.__shelf = shelve.open(fname)
        if self.__shelf is None or self.__shelf_fname != fname:
            self.__shelf_fname = fname
            # slow!
            self.__cached_keys = set(map(float.fromhex, self.__shelf.keys()))
            if len(self.__cached_keys) > 0:
                # add our messages on at the end
                debug('selected shelf already exists: appending at end')
                self.__tzero = time.time() - (max(self.__cached_keys) + 1)
            self.logObject(MessageLoggerSession(fname))
            self.onNewSession()

    def logObject(self, d):
        if self.recordingIsActive():
            t = self.relativeTime()
            debug('shelving object: t=%g' % t, 6)
            while t in self.__cached_keys:
                t = incFloat(t)
            self.__shelf[t.hex()] = d
            self.__cached_keys.add(t)

    def logMessage(self, m):
        mdict = dictFromMessage(m)
        self.logObject(mdict)

    def close(self):
        'This method MUST BE CALLED before destroying the class'
        self.doRecord(False)
        self.stopPlayback()
        self.__shelf.close()
        self.__shelf = None
        self.__cached_keys = None
    
    def convertToCHIL(self, dname, subname=None):
        logger = CHIL.Logger(dname, subname)
        if self.playbackIsActive():
            info('you must stop playback before converting to chil format')
            return
        info('converting to CHIL: %s/%s' % (dname, subname if subname else ''))
        sorted_keys = sorted(list(self.__cached_keys))
        base_time = datetime.datetime(1970,1,1,0,0,0,0)
        try:
            for i in xrange(0, len(sorted_keys)):
                k = sorted_keys[i]
                next_thing = self.__shelf[k.hex()]
                if msg.__class__ and msg.__class__.__name__.endswith('Message'): #pylint: disable=E1101
                    # this is a message
                    debug('t=%g, converting: %s' % (k, next_thing), 5)
                    logger.log(next_thing, base_time + datetime.timedelta(seconds=k))
                elif type(next_thing) == type(dict()):
                    # this is a message saved in the deprecated format
                    m = dictToMessage(next_thing)
                    debug('t=%g, converting: %s' % (k, m), 5)
                    logger.log(m, base_time + datetime.timedelta(seconds=k))
                elif hasattr(next_thing, 'datetime') and hasattr(next_thing, 'info'):
                    # type info of these isn't saved properly??
                    warning('CHIL conversion: session comment "%s" will not be saved' % next_thing.info)
                    base_time = next_thing.datetime
                else:
                    warning('CHIL conversion: %s "%s" will not be saved' % (type(next_thing), next_thing))
        except Exception, e:
            print 'error in conversion!'
            traceback.print_exc()
        finally:
            logger.close()

    def playbackRunloop(self):
        debug('playback started at %fx' % self.playbackRate())
        sorted_keys = sorted(list(self.__cached_keys))
        start_idx = bisect.bisect_left(sorted_keys, self.playbackStartTime())
        sorted_keys = sorted_keys[start_idx:]
        if not len(sorted_keys):
            return
        tstart_recorded = sorted_keys[0]
        tstart_playback = self.relativeTime()
        try:
            for i in xrange(0, len(sorted_keys)):
                if self.playbackHasBeenStopped():
                    info('playback stopped')
                    break
                next_thing = self.__shelf[sorted_keys[i].hex()]
                if msg.__class__ and msg.__class__.__name__.endswith('Message'): #pylint: disable=E1101
                    # this is a message
                    debug('t=%g, sending: %s' % (sorted_keys[i], next_thing), 5)
                    self.node.send(next_thing)
                if type(next_thing) == type(dict()):
                    # this is a message saved in the deprecated format
                    m = dictToMessage(next_thing)
                    debug('t=%g, sending: %s' % (sorted_keys[i], m), 5)
                    self.node.send(m)
                else:
                    info('playback: %s' % next_thing)
                if i != len(sorted_keys) - 1:
                    next_time = sorted_keys[i+1]
                    time_to_sleep_for = (next_time - tstart_recorded) -\
                                        self.playbackRate() * (self.relativeTime() - tstart_playback)
                    #debug('sleeping for %gs' % time_to_sleep_for, 5)
                    if time_to_sleep_for/self.playbackRate() > 10:
                        warning('more than 10 seconds until next message will be sent (%gs)' %
                              (time_to_sleep_for/self.playbackRate()))
                    while time_to_sleep_for > 0 and not self.playbackHasBeenStopped():
                        playback_rate = self.playbackRate()
                        sleep_step = min((time_to_sleep_for/playback_rate, 0.2))
                        time_to_sleep_for -= sleep_step*playback_rate
                        time.sleep(sleep_step)
            if i == len(sorted_keys)-1:
                info('playback complete')
        except Exception, e:
            error('error in playback: ' + str(e))
            raise
        finally:
            self.playbackDidFinish()
        debug('playback finished')

    def onNewSession(self):
        # derived classes can override this
        pass

    def addComment(self, comment_str):
        info('recoding comment "%s"' % comment_str)
        self.logObject(MessageLoggerComment(comment_str))


class CmdPrompt(cmd.Cmd):
    def __init__(self, msg_logger, name):
        self.ml = msg_logger
        self.name = name
        cmd.Cmd.__init__(self)
        self.ruler = ''
        self.undoc_header = 'other commands:'
        self.doc_header = 'available commands:'
        self.setPrompt()
        self.intro = '''
%s, available commands:
    "filename FILENAME" - close the current shelf and use FILENAME as the shelf from now on
    "stop"              - stop recording or playback
    "record"            - stop playback and start recording data
    "playback [RATE] [START_TIME]"
                        - stop recording and start playing back recorded data
                          at RATE * real time, starting at START_TIME into
                          the recoded data
    "chil" BASENAME SUBNAME - convert the current shelf to CHIL format
    "c COMMENT STRING"  - record COMMENT STRING in the shelf
''' % self.name
    def setPrompt(self):
        if self.ml.playbackIsActive():
            self.prompt = '<< '
        else:
            self.prompt = '>> '
    def do_filename(self, filename):
        '"filename FILENAME" - close the current shelf and use FILENAME as the shelf from now on'
        try:
            self.ml.setFile(filename)
        except:
            print 'invalid filename "%s"' % filename
        self.setPrompt()
    def complete_filename(self, filename):
        # TODO: filename completions
        return []
    def do_stop(self, l):
        '"stop"              - stop recording or playback'
        if len(l):
            print '"stop" takes no arguments'
            return
        self.ml.stopPlayback()
        self.ml.doRecord(False)
        self.setPrompt()
    def do_record(self, l):
        '"record"            - stop playback and start recording data'
        if len(l):
            print '"record" takes no arguments'
            return
        self.ml.doRecord(True)
        self.setPrompt()
    def do_chil(self, l):
        basename, subname = l.split()
        self.ml.convertToCHIL(basename, subname)
    def do_playback(self, l):
        '''"playback [RATE] [START_TIME]"
                        - stop recording and start playing back recorded data
                          at RATE * real time, starting at START_TIME into
                          the recoded data'''
        try:
            rate, start = l.split()
        except ValueError: 
            rate = l
            start = ''
        start_time = 0.0
        if len(rate):
            try:
                self.ml.setPlaybackRate(float(rate))
            except ValueError, e:
                print 'could not set playback rate: "%s" (value should be a number)' % rate
                return
        if len(start):
            try:
                start_time = float(start)
            except ValueError, e:
                print 'could not set start time: "%s" (value should be a number of seconds)' % start
                return
        self.ml.startPlayback(start_time)
        self.setPrompt()
    def do_c(self, line):
        '"c COMMENT STRING"  - record COMMENT STRING in the shelf'
        self.ml.addComment(line)
    def do_EOF(self, line):
        return True
    def emptyline(self):
        self.onecmd('help')
        return False
