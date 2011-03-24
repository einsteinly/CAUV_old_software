import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error


Default_Groups_To_Join = (
    'control',
    'sonarctl'
)

Default_Messages_To_Watch = (
    'BearingAutopilotParams',
    'DepthAutopilotParams',
    'DepthCalibration',
    'PitchAutoPilotParams',
    'SetMotorMap',
    'SonarControl'
)


class PersistentSetting:
    def __init__(self, setMethodName, value):
        self.recvMethodName = recvMethodName
        self.sendMethodName = sendMethodName
        self.value = value

class OnAnyMessage:
    def __init__(self, msg_name):
        self.message_name = msg_name
    def onMessage(self, m):
        debug('onMessage expect %s have %s' % (self.message_name, m))

class PersistObserver(messaging.MessageObserver):
    def __init__(self, node, groups, watch_list):
        messaging.MessageObserver.__init__(self)
        self.__node = node
        for group in groups:
            node.join(group)
        for message in watch_list:
            self.attachOnMessageFunc(message)
        node.addObserver(self)
    
    def attachOnMessageFunc(self, name):
        func = OnAnyMessage(name).onMessage
        setattr(self, 'on%sMessage%s' % name, func)


if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-f', '--persistence-file', dest='fname', default='persistent-settings.pickle',
                 action='store', help='file name to save/load from')
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help='do broadcast messages for saved settings')

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)

    cauv_node = node.Node("persist")


