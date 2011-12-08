#!/usr/bin/env python2.7
#pylint: disable=E1101

# Standard Library
import traceback
import argparse

# CAUV
import cauv.messaging as msg
import cauv.node as node 
import messageLogger
from cauv.debug import debug, info, warning, error
from utils.hacks import injectBase

class TelemetryLogger(object):
    def __init__(self, cauv_node, fname, do_record):
        # do not ask about, or change, this line:
        super(TelemetryLogger, self).__init__(cauv_node, fname, do_record)
        self.node.join('gui')
        self.node.join('telemetry')

    def onNewSession(self):
        pass
    
    # def onControllerStateMessage(self, m):
    #    self.logMessage(m)

    def onMotorStateMessage(self, m):
        self.logMessage(m)

    def onTelemetryMessage(self, m):
        self.logMessage(m)

class TelemetryLoggerCmdPrompt(messageLogger.CmdPrompt):
    def __init__(self, logger):
        messageLogger.CmdPrompt.__init__(self, logger, 'Telemetry Logger')

def telemetryLoggerMainLoop(cauv_node, opts):
    if opts.fname.endswith('.shelf'):
        warning('using deprecated logging format because you specified a "shelf" filename') 
        LoggerClass = injectBase(TelemetryLogger, messageLogger._DeprecatedShelfLogger)
    else:
        LoggerClass = injectBase(TelemetryLogger, messageLogger.CHILLogger)
    tl = LoggerClass(cauv_node, opts.fname, not opts.no_record)
    cli = TelemetryLoggerCmdPrompt(tl)
    playback_is_active = False
    try:
        cli.cmdloop()
    except KeyboardInterrupt:
        pass
    finally:
        # this gets run even if we were killed (software-killed that is,
        # nothing can save us from the kill-switch...)
        tl.close()
        info('closed down properly')
    info('exiting...')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--log-fname', dest='fname',
                 default='./default.chil',
                 action='store', help='file to load/save telemetry data from')
    p.add_argument('-n', '--no-record', dest='no_record', default=False,
                 action='store_true', help="Don't start in recording mode")

    opts, args = p.parse_known_args()

    cauv_node = node.Node("py-tlog",args) 
    try:
        telemetryLoggerMainLoop(cauv_node, opts)
    finally:
        cauv_node.stop()


