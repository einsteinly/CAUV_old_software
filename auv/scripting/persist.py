#!/usr/bin/env python2.7

import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import contextlib
import shelve
import traceback
import argparse
import time

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

Ignore_Message_Attrs = (
    'group',
    'chil',
    'msgId'
)

def sendSavedMessages(node, shelf):
    info('restoring saved settings...')
    for mad_id in shelf:
        msg_name = mad_id.split(':')[0]
        if not msg_name in Default_Messages_To_Watch:
            continue
        try:
            attrs = shelf[mad_id]
            info('restoring saved %s: %s' % (msg_name, attrs))
            m = dictToMessage(msg_name, attrs)
            node.send(m)
        except Exception, e:
            warning('Exception: %s\nattempting to continue...' %
                    traceback.format_exc())
    info('restore complete')

def dictFromMessage(message):
    attrs = message.__class__.__dict__
    r = {}
    for k in attrs:
        if not k.startswith('__') and not k in Ignore_Message_Attrs:
            # hack hack hack hack hack
            # I'm sure there is a simpler way of doing this, but m.__dict__ is
            # unhelpfully empty...
            r[k] = attrs[k].__get__(message)
    return r

def dictToMessage(message_name, attr_dict):
    return getattr(msg, message_name+'Message')(**attr_dict)

class OnAnyMessage:
    def __init__(self, msg_name, shelf):
        self.message_name = msg_name
        self.shelf = shelf
    def onMessage(self, m):
        debug('onMessage expect %sMessage have %s' % (self.message_name, m))
        if self.message_name in Message_ID_Fields: 
            msg_id_field = getattr(m, str(Message_ID_Fields[self.message_name]))
            save_as_id = "%s:%s" % (self.message_name, msg_id_field)
        else:
            save_as_id = self.message_name
        self.shelf[save_as_id] = dictFromMessage(m)
        # don't take any chances
        self.shelf.sync()

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
            sendSavedMessages(self.__node, self.__shelf)

    def attachOnMessageFunc(self, name):
        func = OnAnyMessage(name, self.__shelf).onMessage
        setattr(self, 'on%sMessage' % name, func)

def persistMainLoop(cauv_node, shelf, auto, silent = False):
    po = PersistObserver(cauv_node, shelf, Default_Messages_To_Watch, auto)
    help_str = 'available commands: "set" - broadcast saved '+\
               'parameters, "exit" - stop monitoring and exit, '+\
               '"auto on" / "auto off" - control automatic ' +\
               'parameter broadcast on membership change'
    try:
        if silent:
            while True:
                time.sleep(10000)
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

    except KeyboardInterrupt:
        pass
    finally:
        # this gets run even if we were killed (software-killed that is,
        # nothing can save us from the kill-switch...)
        shelf.close()
        info('closed the shelf')
    info('exiting...')

def shelve_open(filename):
    #hack because bsddb is broken by default on Arch
    # 'depreciated module' doesn't mean you should hack it out
    # breaking perfectly good modules in the process...
    # see https://bugs.archlinux.org/task/25058
    try:
        shelf = shelve.open(filename)
    except ImportError:
        import bsddb3
        _db = bsddb3.hashopen(filename)
        shelf = shelve.Shelf(_db)
    return shelf

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--persistence-file', dest='fname',
                 default='./settings-persistence.shelf',
                 action='store', help='file name to save/load from (python shelf)')
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help='immediately broadcast messages for saved settings')
    p.add_argument('-n', '--no-auto', dest='auto', default=True,
                 action='store_false', help="don't automatically set parameters" +\
                 "when CAUV Nodes connect to the messaging system")
    p.add_argument('-s', '--silent', action='store_true',
                  help="don\'t prompt for commands")

    opts, args = p.parse_known_args()

    cauv_node = node.Node("persist",args)
    shelf = shelve_open(opts.fname)
    
    if opts.restore:
        sendSavedMessages(cauv_node, shelf)
    try:
        persistMainLoop(cauv_node, shelf, opts.auto, opts.silent)
    finally:
        print 'finally...'
        cauv_node.stop()

