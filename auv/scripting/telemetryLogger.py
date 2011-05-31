import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import messageLogger

import traceback
import optparse

class TelemetryLogger(messageLogger.Logger):
    def __init__(self, cauv_node, shelf_fname, do_record):
        messageLogger.Logger.__init__(self, cauv_node, shelf_fname, do_record)
        self.node.join('gui')
        self.node.join('telemetry')

    def onNewSession(self):
        pass
    
    # def onControllerStateMessage(self, m):
    #    self.shelveMessage(m)

    def onMotorStateMessage(self, m):
        self.shelveMessage(m)

    def onTelemetryMessage(self, m):
        self.shelveMessage(m)

class TelemetryLoggerCmdPrompt(messageLogger.CmdPrompt):
    def __init__(self, logger):
        messageLogger.CmdPrompt.__init__(self, logger, 'Telemetry Logger')

def telemetryLoggerMainLoop(cauv_node, opts):
    tl = TelemetryLogger(cauv_node, opts.fname, not opts.no_record)
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
    p = optparse.OptionParser()
    p.add_option('-f', '--log-shelf', dest='fname',
                 default='./telemetry.shelf',
                 action='store', help='file to load/save telemetry data from')
    p.add_option('-n', '--no-record', dest='no_record', default=False,
                 action='store_true', help="Don't start in recording mode")

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)
   
    cauv_node = node.Node("py-tlog") 
    telemetryLoggerMainLoop(cauv_node, opts)


