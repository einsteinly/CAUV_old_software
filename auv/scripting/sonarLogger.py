import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import contextlib
import shelve
import traceback
import optparse
import time
import datetime
import sys
import math
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

class NewSonarDataSession:
    def __init__(self, info_str):
        self.datetime = datetime.datetime.now()
        self.info = info_str

    def __repr__(self):
        return 'New sonar session (%s), recorded at %s' % (
            self.info, self.datetime.strftime(Datetime_Format)
        )

class SonarDataComment:
    def __init__(self, comment_str):
        self.datetime = datetime.datetime.now()        
        self.comment = comment_str

    def __repr__(self):
        return 'Comment: %s (recorded at %s)' % (
            self.comment, self.datetime.strftime(Datetime_Format)
        )

class SonarLogger(msg.MessageObserver):
    def __init__(self, cauv_node, shelf_fname, do_record):
        msg.MessageObserver.__init__(self)
        self.__node = cauv_node
        self.__recording = False
        self.__shelf = None
        self.__cached_keys = set()
        self.__shelf_fname = None # set by setShelf
        self.__sonar_params = None
        self.__tzero = time.time()
        self.__node.join('sonarctl')
        self.__node.join('sonarout')
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
            self.shelveObject(NewSonarDataSession(fname))
            self.shelveObject(self.__sonar_params)
    
    def addComment(self, comment_str):
        info('recoding comment "%s"' % comment_str)
        self.shelveObject(SonarDataComment(comment_str))

    def doRecord(self, do_record):
        if self.__recording == do_record:
            return
        if do_record == True:
            self.__node.addObserver(self)
        else:
            self.__node.removeObserver(self)
            self.__shelf.sync()

    def relativeTime(self):
        return time.time() - self.__tzero

    def shelveObject(self, d):
        t = self.relativeTime()
        while t in self.__cached_keys:
            t = self.relativeTime()
        self.__shelf[t.hex()] = d
        self.__cached_keys.add(t)

    def onSonarDataMessage(self, m):
        mdict = dictFromMessage(m)
        self.shelveObject(mdict)

    def onSonarControlMessage(self, m):
        mdict = dictFromMessage(m)
        debug('sonar control message: %s' % mdict)
        self.__sonar_params = mdict
        self.shelveObject(mdict)

class sonarLoggerCmdPrompt(cmd.Cmd):
    def __init__(self, sonar_logger):
        self.sl = sonar_logger
        cmd.Cmd.__init__(self)
        self.ruler = ''
        self.undoc_header = 'other commands:'
        self.doc_header = 'available commands:'
        self.setPrompt()
        self.intro = '''
Sonar Data Logger, available commands:
    "filename FILENAME" - close the current shelf and use FILENAME as the shelf from now on
    "stop"              - stop recording or playback
    "record"            - stop playback and start recording data
    "playback"          - stop recording and start playing back recorded data
    "c COMMENT STRING"  - record COMMENT STRING in the shelf
'''
    def setPrompt(self):
        if self.sl.playbackIsActive():
            self.prompt = '<< '
        else:
            self.prompt = '>> '
    def do_filename(self, filename):
        '"filename FILENAME" - close the current shelf and use FILENAME as the shelf from now on'
        try:
            self.sl.setShelf(filename)
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
        self.sl.doRecord(False)
    def do_record(self, l):
        '"record"            - stop playback and start recording data'
        if len(l):
            print '"record" takes no arguments'
            return
        self.sl.doRecord(True)
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
        self.sl.addComment(line)
    def do_EOF(self, line):
        return True
    def emptyline(self):
        self.onecmd('help')
        return False



def sonarLoggerMainLoop(cauv_node, opts):
    sl = SonarLogger(cauv_node, opts.fname, not opts.no_record)
    cli = sonarLoggerCmdPrompt(sl)
    playback_is_active = False
    try:
        cli.cmdloop()
    except KeyboardInterrupt:
        pass
    finally:
        # this gets run even if we were killed (software-killed that is,
        # nothing can save us from the kill-switch...)
        sl.close()
        info('closed down properly')
    info('exiting...')

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-f', '--log-shelf', dest='fname',
                 default='./sonar.shelf',
                 action='store', help='file to load/save sonar data lines from')
    p.add_option('-n', '--no-record', dest='no_record', default=False,
                 action='store_true', help="Don't start in recording mode")

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)
   
    cauv_node = node.Node("py-slog") 
    sonarLoggerMainLoop(cauv_node, opts)

