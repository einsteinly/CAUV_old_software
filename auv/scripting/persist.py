#!/usr/bin/env python

import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

import contextlib
import shelve
import traceback
import optparse

Default_Groups_To_Join = (
    'control',
    'sonarctl'
)

Default_Messages_To_Watch = (
    'BearingAutopilotParams',
    'DepthAutopilotParams',
    'DepthCalibration',
    'PitchAutopilotParams',
    'SetMotorMap',
    'MotorRampRate',
    'SonarControl'
)

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
    return r

def dictToMessage(message_name, attr_dict):
    return getattr(msg, message_name+'Message')(**attr_dict)

class OnAnyMessage:
    def __init__(self, msg_name, shelf):
        self.message_name = msg_name
        self.shelf = shelf
    def onMessage(self, m):
        debug('onMessage expect %sMessage have %s' % (self.message_name, m))
        self.shelf[self.message_name] = dictFromMessage(m)
        # don't take any chances
        self.shelf.sync()

class PersistObserver(msg.MessageObserver):
    def __init__(self, node, shelf, groups, watch_list):
        msg.MessageObserver.__init__(self)
        self.__node = node
        self.__shelf = shelf
        for group in groups:
            node.join(group)
        for message in watch_list:
            debug('Listening for %s messages' % message)
            self.attachOnMessageFunc(message)
        node.addObserver(self)
    
    def attachOnMessageFunc(self, name):
        func = OnAnyMessage(name, self.__shelf).onMessage
        setattr(self, 'on%sMessage' % name, func)

def sendSavedMessages(node, shelf):
    info('restoring saved settings...')
    for msg_name in shelf:
        try:
            attrs = shelf[msg_name]
            info('restoring saved %s: %s' % (msg_name, attrs))
            m = dictToMessage(msg_name, attrs)
            node.send(m)
        except Exception, e:
            warning('Exception: %s\nattempting to continue...' %
                    traceback.format_exc())
    info('restore complete')

def persistMainLoop(cauv_node, shelf):
    po = PersistObserver(cauv_node, shelf, Default_Groups_To_Join,
                         Default_Messages_To_Watch)
    help_str = 'available commands: "set" - broadcast saved '+\
               'parameters, "exit" - stop monitoring and exit'
    try:
        print help_str
        while True:
            s = raw_input('\n> ')
            s = s.lower().strip()
            if s == 'exit':
                break
            if s == 'set':
                sendSavedMessages(cauv_node, shelf)
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

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-f', '--persistence-file', dest='fname',
                 default='/home/cauv/settings-persistence.shelf',
                 action='store', help='file name to save/load from (python shelf)')
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help='immediately broadcast messages for saved settings')

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)

    cauv_node = node.Node("persist")
    shelf = shelve.open(opts.fname)
    
    if opts.restore:
        sendSavedMessages(cauv_node, shelf)
    persistMainLoop(cauv_node, shelf)

