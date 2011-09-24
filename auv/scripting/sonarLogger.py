#!/usr/bin/env python

# Standard Library
import traceback
import optparse

# CAUV
import cauv.messaging as msg
import cauv.node as node 
import messageLogger
from cauv.debug import debug, info, warning, error
from utils.hacks import injectBase

class SonarLogger(object):
    def __init__(self, cauv_node, fname, do_record):
        self.__sonar_params = None
        # do not ask about, or change, this line:
        super(self.__class__, self).__init__(cauv_node, fname, do_record)
        self.node.join('sonarctl')
        self.node.join('sonarout')

    def onNewSession(self):
        self.logObject(self.__sonar_params)

    def onSonarDataMessage(self, m):
        self.logMessage(m)

    def onSonarControlMessage(self, m):
        self.logMessage(m)
        mdict = messageLogger.dictFromMessage(m)
        debug('sonar control message: %s' % mdict)
        self.__sonar_params = mdict

class SonarLoggerCmdPrompt(messageLogger.CmdPrompt):
    def __init__(self, sonar_logger):
        messageLogger.CmdPrompt.__init__(self, sonar_logger, 'Sonar Logger')

def sonarLoggerMainLoop(cauv_node, opts):
    if opts.fname.endswith('.shelf'):
        warning('using deprecated logging format because you specified a "shelf" filename') 
        LoggerClass = injectBase(SonarLogger, messageLogger._DeprecatedShelfLogger)
    else:
        LoggerClass = injectBase(SonarLogger, messageLogger.CHILLogger)
    sl = LoggerClass(cauv_node, opts.fname, not opts.no_record)
    cli = SonarLoggerCmdPrompt(sl)
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
    p.add_option('-f', '--log-file', dest='fname',
                 default='./default.chil',
                 action='store', help='file to load/save sonar data lines from')
    p.add_option('-n', '--no-record', dest='no_record', default=False,
                 action='store_true', help="Don't start in recording mode")

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)
   
    cauv_node = node.Node("py-slog") 
    try:
        sonarLoggerMainLoop(cauv_node, opts)
    finally:
        cauv_node.stop()

