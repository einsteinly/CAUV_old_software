#!/usr/bin/env python2.7

# Standard Library
import traceback
import argparse

# CAUV
import cauv.messaging as msg
import cauv.node as node 
import messageLogger
from cauv.debug import debug, info, warning, error
from utils.hacks import injectBase

class SonarLogger(object):
    def __init__(self, cauv_node, fname, do_record):
        self.__sonar_params_message = None
        self.__gemini_params_message = None
        # do not ask about, or change, this line:
        super(self.__class__, self).__init__(cauv_node, fname, do_record)
        self.node.join('sonarctl')
        self.node.join('sonarout')

        # set up messages to log:
        # methods defined like this to short-circuit one function call...
        # perhaps this is a little bit of unnecessary optimisation, but these
        # functions do get called a *lot*
        self.onSonarDataMessage = self.logMessage
        self.onSonarImageMessage = self.logMessage
        self.onSpeedOfSoundMessage = self.logMessage
        #self.onTelemetryMessage = self.logMessage # Log telemetry so that we have orientation data to match the sonar images
        self.onGeminiStatusMessage = self.logMessage
        self.onSonarControlMessage = self.logMessage
        # These are defined normally because they sometimes print information
        # and they aren't sent very frequently anyway
        #self.onSonarControlMessage = self.logMessage
        #self.onGeminiControlMessage = self.logMessage

    def onNewSession(self):
        if self.__sonar_params_message is not None:
            self.logMessage(self.__sonar_params_message)
        if self.__gemini_params_message is not None:
            self.logMessage(self.__gemini_params_message)

    def onSonarControlMessage(self, m):
        self.logMessage(m)
        debug('sonar control message: %s' % m)
        self.__sonar_params_message = m

    def onGeminiControlMessage(self, m):
        self.logMessage(m)
        debug('gemini control message: %s' % m)
        self.__gemini_params_message = m

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
    sl.profile_playback = opts.do_profile
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
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--log-file', dest='fname',
                 default='./default.chil',
                 action='store', help='file to load/save sonar data lines from')
    p.add_argument('-n', '--no-record', dest='no_record', default=False,
                 action='store_true', help="Don't start in recording mode")
    p.add_argument('-p', '--profile', dest='do_profile', default=False,
                 action='store_true', help='use cProfile to run everything')
    opts, args = p.parse_known_args()

    cauv_node = node.Node("py-slog",args)
    try: 
        if opts.do_profile:
            import cProfile
            cProfile.run('sonarLoggerMainLoop(cauv_node, opts)', 'sonarLogger.profile')
        else:
            sonarLoggerMainLoop(cauv_node, opts)
    finally:
        cauv_node.stop()

