import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import shelve
import datetime
import sys
import math
import time
import cmd

#TODO: there's quite a lot of code duplication here, we should make a CAUV
# python utility library with stuff like this in it

Datetime_Format = '%a %d %b %Y %H:%M:%S'

Ignore_Message_Attrs = (
    'group'
)

def dictFromMessage(message):
    attrs = message.__class__.__dict__
    r = {}
    for k in attrs:
        if not k.startswith('__') and not k in Ignore_Message_Attrs:
            # hack hack hack hack hack
            # I'm sure there is a simpler way of doing this, but m.__dict__ is
            # unhelpfully empty...
            r[k] = attrs[k].__get__(message)
    r['__message_name__'] = message.__name__
    return r

def dictToMessage(attr_dict):
    return getattr(msg, attr_dict['__message_name__'])(**attr_dict)

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

class Logger(msg.MessageObserver):
    def __init__(self, cauv_node, shelf_fname, do_record):
        msg.MessageObserver.__init__(self)
        self.node = cauv_node
        self.__recording = False
        self.__shelf = None
        self.__cached_keys = set()
        self.__shelf_fname = None # set by setShelf
        self.__tzero = time.time()
        self.setShelf(shelf_fname)
        self.doRecord(do_record)
    
    def close(self):
        'This method MUST BE CALLED before destroying the class'
        self.doRecord(False)
        self.__shelf.close()
        self.__shelf = None
        self.__cached_keys = None

    def playbackIsActive(self):
        # TODO: message playback
        return False

    def setShelf(self, fname):
        if self.__shelf is not None and self.__shelf_fname != fname:
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
                self.__tzero = time.time() - max(self.__cached_keys)
            self.shelveObject(MessageLoggerSession(fname))
            self.onNewSession()
    
    def onNewSession(self):
        # derived classes can override this
        pass

    def addComment(self, comment_str):
        info('recoding comment "%s"' % comment_str)
        self.shelveObject(MessageLoggerComment(comment_str))

    def doRecord(self, do_record):
        if self.__recording == do_record:
            return
        if do_record == True:
            self.node.addObserver(self)
        else:
            self.node.removeObserver(self)
            self.__shelf.sync()

    def relativeTime(self):
        return time.time() - self.__tzero

    def shelveObject(self, d):
        t = self.relativeTime()
        while t in self.__cached_keys:
            t = incfloat(t)
        self.__shelf[t.hex()] = d
        self.__cached_keys.add(t)

    def shelveMessage(self, m):
        mdict = dictFromMessage(m)
        self.shelveObject(mdict)

        
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
    "playback"          - stop recording and start playing back recorded data
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
            self.ml.setShelf(filename)
        except:
            print 'invalid filename "%s"' % filename
    def complete_filename(self, filename):
        # TODO: filename completions
        return []
    def do_stop(self, l):
        '"stop"              - stop recording or playback'
        if len(l):
            print '"stop" takes no arguments'
            return
        self.ml.doRecord(False)
    def do_record(self, l):
        '"record"            - stop playback and start recording data'
        if len(l):
            print '"record" takes no arguments'
            return
        self.ml.doRecord(True)
        self.setPrompt()         
    def do_playback(self, l):
        '"playback"          - stop recording and start playing back recorded data'
        if len(l):
            print '"playback" takes no arguments'
            return
        print "data playback isn't implemented yet"
        self.setPrompt() 
    def do_c(self, line):
        '"c COMMENT STRING"  - record COMMENT STRING in the shelf'
        self.ml.addComment(line)
    def do_EOF(self, line):
        return True
    def emptyline(self):
        self.onecmd('help')
        return False
