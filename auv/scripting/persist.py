#!/usr/bin/env python2.7

import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import traceback
import argparse
import time
import inspect
import utils.jsonshelf

Default_Messages_To_Watch = (
    'BearingAutopilotParams',
    'DepthAutopilotParams',
    'DepthCalibration',
    'PitchAutopilotParams',
    'SetMotorMap',
    'MotorRampRate',
    'SonarControl',
    'GeminiControl'
)

# Consider messages with different values in ID fields to be different
# messages:
Message_ID_Fields = {
    'SetMotorMap': 'motor'
}

def dictFromMessage(message):
    """Turn a message into a dictionary suitable for JSON serialisation"""
    # now 20%300% hackier!
    # You know you're in trouble when 30% of the variables are __dunders__...
    if message.__class__.__name__ not in msg.__dict__.keys():
        #not a message / struct, just return original
        return message
    ret = {}
    ret["__name__"] = message.__class__.__name__
    try:
        # Assume it's an enum...
        val = int(message)
        #No, message.name doesn't work
        ret["value"] = message.values[val].name
    except TypeError:
        # OK, it isn't
        contents = {}
        for name, member in inspect.getmembers(message.__class__,
                                               lambda x: isinstance(x, property)):
            contents[name] = dictFromMessage(member.__get__(message))
        ret["contents"] = contents
    #sanity check results
    dictToMessage(ret)
    return ret

def dictToMessage(message_dict):
    # This is much less hacky and likely to break than the inverse
    name = message_dict['__name__']
    if 'value' in message_dict:
        enum_cls = getattr(msg, name)
        return getattr(enum_cls, message_dict['value'])
    contents = dict(message_dict["contents"])
    for attr in contents:
        try:
            contents[attr] = dictToMessage(contents[attr])
        except (KeyError, TypeError):
            pass
    # boost::python doesn't like unicode...
    contents = {key.encode('ascii', 'ignore') : val for key, val in contents.iteritems()}
    return getattr(msg, name)(**contents)

def sendSavedMessages(node, shelf):
    info('restoring saved settings...')
    for mad_id in shelf:
        msg_name = mad_id.split(':')[0]
        if not msg_name in Default_Messages_To_Watch:
            continue
        try:
            msg_dict = shelf[mad_id]
            info('restoring saved %s: %s' % (msg_name, msg_dict))
            m = dictToMessage(msg_dict)
            node.send(m)
        except Exception, e:
            warning('Exception: %s\nattempting to continue...' %
                    traceback.format_exc())
    info('restore complete')

class OnAnyMessage:
    def __init__(self, msg_name, shelf):
        self.message_name = msg_name
        self.shelf = shelf
    def onMessage(self, m):
        debug('onMessage expect %sMessage have %s' % (self.message_name, m))
        if self.message_name in Message_ID_Fields: 
            msg_id_field = getattr(m, str(Message_ID_Fields[self.message_name]))
            save_as_id = "%s_%s" % (self.message_name, msg_id_field)
        else:
            save_as_id = self.message_name
        self.shelf[save_as_id] = dictFromMessage(m)

class PersistObserver(msg.MessageObserver):
    def __init__(self, node, shelf, watch_list, auto):
        msg.MessageObserver.__init__(self)
        self.__node = node
        self.__shelf = shelf
        self.__auto = auto
        if auto:
            node.subMessage(msg.MembershipChangedMessage())
        for message in watch_list:
            debug('Listening for %s messages' % message)
            node.subMessage(getattr(msg, message+'Message')())
            self.attachOnMessageFunc(message)
        node.addObserver(self)

    def setAuto(self, v):
        self.__auto = v
    
    def onMembershipChangedMessage(self, m):
        if self.__auto:
            info('received membership changed message, broadcasting parameters...')
            time.sleep(0.3) #blergh
            sendSavedMessages(self.__node, self.__shelf)

    def attachOnMessageFunc(self, name):
        func = OnAnyMessage(name, self.__shelf).onMessage
        setattr(self, 'on%sMessage' % name, func)

def persistMainLoop(cauv_node, shelf, auto, silent = False, read_only = False, broadcast_rate = None):
    po = PersistObserver(cauv_node, shelf, Default_Messages_To_Watch if not read_only else [], auto)
    help_str = 'available commands: "set" - broadcast saved '+\
               'parameters, "exit" - stop monitoring and exit, '+\
               '"auto on" / "auto off" - control automatic ' +\
               'parameter broadcast on membership change'
    if silent:
        if broadcast_rate is None:
            while True:
                time.sleep(1000)
        else:
            while True:
                time.sleep(broadcast_rate)
                sendSavedMessages(cauv_node, shelf)
    else:
        print help_str
        while True:
            s = raw_input('\n> ')
            s = s.lower().strip()
            if s == 'exit':
                break
            elif s == 'set':
                sendSavedMessages(cauv_node, shelf)
            elif s == 'auto on':
                po.setAuto(True)
            elif s == 'auto off':
                po.setAuto(False)
            else:
                print help_str
    info('exiting...')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--persistence-dir', dest='fname',
                 default='./persist',
                 action='store', help='directory to load/store messages from')
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help='immediately broadcast messages for saved settings')
    p.add_argument('-n', '--no-auto', dest='auto', default=True,
                 action='store_false', help="don't automatically set parameters" +\
                 "when CAUV Nodes connect to the messaging system")
    p.add_argument('-s', '--silent', action='store_true', help="don\'t prompt for commands")
    p.add_argument('-o', '--read-only', help="don't save broadcasted messages", action = 'store_true')
    p.add_argument('-e', '--every', help='broadcast every n seconds', type=int)

    opts, args = p.parse_known_args()

    cauv_node = node.Node("persist",args)
    shelf = utils.jsonshelf.JSONShelf(opts.fname)
    
    if opts.restore:
        sendSavedMessages(cauv_node, shelf)
    persistMainLoop(cauv_node, shelf, opts.auto, opts.silent, opts.read_only, opts.every)
