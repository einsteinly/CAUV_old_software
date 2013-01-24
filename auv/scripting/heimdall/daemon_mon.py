#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import cauv.messaging as msgs
from cauv.debug import debug, info, warning, error

import urwid
import zmq
import utils.zmqfuncs
import argparse
import re
import time
import collections

message_map = dict(((msgs.__dict__[m].msgId, m) for m in msgs.__dict__ if m.endswith("Message")))

parser = argparse.ArgumentParser(description = 'Monitor vehicle_daemon status')
parser.add_argument('--vehicle', '-v', help='daemon vehicle name')
parser.add_argument('--ipc-dir', '-i', help='ipc directory')

opts = parser.parse_args()

d_ctrl = utils.zmqfuncs.DaemonControl(opts.vehicle, opts.ipc_dir)

class MessageStats:
    def __init__(self, m_id = None, sub_in = False, sub_out = False, 
                 n_in = 0, n_out = 0, rate_in = 0, rate_out = 0, bytes_in = 0, bytes_out = 0):
        self.m_id = id
        self.n_in = n_in
        self.n_out = n_out
        self.bytes_in = bytes_in
        self.bytes_out = bytes_out
        self.rate_in = rate_in
        self.rate_out = rate_out
        self.sub_in = sub_in
        self.sub_out = sub_out

def format_bytes(n_bytes):
    suffixes = ['B','kB','MB','GB']
    for suffix in suffixes:
        if n_bytes > 1000:
            n_bytes /= 1000
        else:
            return '{} {}'.format(n_bytes, suffix)

class MessageStatsTracker:
    def __init__(self):
        self.stats_hist = []
        self.sort_keys = [
            (lambda x: x.m_id, 'message id', False),
            (lambda x: message_map[x.m_id], 'message name', False),
            (lambda x: x.n_in, 'messages in', True),
            (lambda x: x.n_out, 'messages out', True),
            (lambda x: x.bytes_in, 'bytes in', True),
            (lambda x: x.bytes_out, 'bytes out', True),
            (lambda x: x.rate_in, 'rate in', True),
            (lambda x: x.rate_out, 'rate out', True),
        ]
        self.sort_key = 0

    def get_message_stats(self, dt = 1, n_avg = 1):
        stats = d_ctrl.run_cmd('STATS')
        subs = d_ctrl.run_cmd('SUBS')
        messages = collections.defaultdict(MessageStats)
        for msg in stats['net_to_local']:
            if msg is None:
                continue
            messages[msg['id']].n_in = msg['messages']
            messages[msg['id']].bytes_in = msg['bytes']
        for msg in stats['local_to_net']:
            if msg is None:
                continue
            messages[msg['id']].n_out = msg['messages']
            messages[msg['id']].bytes_out = msg['bytes']
        for msg in subs['net_to_local']:
            if msg is None:
                continue
            messages[msg].sub_in = True
        for msg in subs['local_to_net']:
            if msg is None:
                continue
            messages[msg].sub_out = True

        for msg_id in messages:
            messages[msg_id].m_id = msg_id
            m = messages[msg_id]
            if len(self.stats_hist) > 0:
                if msg_id in self.stats_hist[-1]:
                    m.rate_in = (m.bytes_in - self.stats_hist[-1][msg_id].bytes_in)/dt
                    m.rate_out = (m.bytes_out - self.stats_hist[-1][msg_id].bytes_out)/dt

        message_rows = []
        sort_meth = self.sort_keys[self.sort_key]
        for m in sorted(messages.values(), key = sort_meth[0], reverse = sort_meth[2]):
            attr = 'no_sub'
            if m.sub_in:
                attr = 'sub_in'
            if m.sub_out:
                attr = 'sub_out'
            if m.sub_in and m.sub_out:
                attr = 'sub_both'
            row = [(attr,      message_map[m.m_id]),
                   ('val_in',  str(m.n_in)),
                   ('val_out', str(m.n_out)),
                   ('val_in',  format_bytes(m.bytes_in)), 
                   ('val_out', format_bytes(m.bytes_out)),
                   ('val_in',  format_bytes(m.rate_in) + '/s'),
                   ('val_out', format_bytes(m.rate_out) + '/s')]

            message_rows.append(row)
        self.stats_hist.append(messages)
        if len(self.stats_hist) > n_avg:
            del self.stats_hist[0]
        col_widths = [max((len(s[1]) for s in x)) for x in zip(*message_rows)]
        format_strs = ["{{:<{}}}"] + ["  {{:>{}}}"] * (len(col_widths) - 1)
        format_strs = [f[0].format(f[1]) for f in zip(format_strs, col_widths)]
        
        text_rows = []
        for row in message_rows:
            text = urwid.Text([(t[0][0], t[1].format(t[0][1])) for t in zip(row, format_strs)])
            text_rows.append(text)
        return text_rows

def get_connection_box(socket):
    connections = urwid.SimpleListWalker([])
    conn_list_box = urwid.ListBox(connections)
    title_box = urwid.LineBox(conn_list_box, title = socket)
    title_box.conn_list = connections
    title_box.socket_name = socket
    return title_box

def get_connections():
    connections = d_ctrl.run_cmd('CONNECTIONS')
    socket_conns = {}
    for socket in connections:
        conn_strs = []
        for connection in (x['string'] for x in connections[socket]['binds']):
            conn_strs.append(urwid.Text(('bind', connection)))
        for connection in (x['string'] for x in connections[socket]['connections']):
            conn_strs.append(urwid.Text(('connect', connection)))
        socket_conns[socket] = conn_strs
    return socket_conns

connection_pane_list = urwid.SimpleListWalker([])
connections = d_ctrl.run_cmd('CONNECTIONS')
for socket in connections:
    connection_pane_list.append(get_connection_box('{}'.format(socket)))

help_text = urwid.Text('')

conn_list_pile = urwid.Pile(connection_pane_list)

message_list = urwid.SimpleListWalker([])
message_list._explicit_modified = message_list._modified 
message_list._modified = lambda: None
message_stats_list = urwid.ListBox(message_list)
message_stats_box = urwid.LineBox(message_stats_list)
message_stats_box.title_fmt = 'message stats (message / messages in/out / bytes in/out / bitrate in/out) s: {}'

columns = urwid.Columns([('fixed', 50, conn_list_pile), message_stats_box])

stats_tracker = MessageStatsTracker()
message_stats_box.set_title(message_stats_box.title_fmt.format(stats_tracker.sort_keys[0][1]))

def handle_input(input):
    if input == 'q':
        raise urwid.ExitMainLoop()
    elif input == 's':
        stats_tracker.sort_key += 1
        if stats_tracker.sort_key >= len(stats_tracker.sort_keys):
            stats_tracker.sort_key = 0
        message_stats_box.set_title(message_stats_box.title_fmt.format(stats_tracker.sort_keys[stats_tracker.sort_key][1]))
    elif input == 'a':
        stats_tracker.sort_key -= 1
        if stats_tracker.sort_key < 0:
            stats_tracker.sort_key = len(stats_tracker.sort_keys) - 1
        message_stats_box.set_title(message_stats_box.title_fmt.format(stats_tracker.sort_keys[stats_tracker.sort_key][1]))
    elif input == 'r':
        global rd
        rd = '''eNrdmEl6wzAIhfe9gjc6gjwvepScgftvm8GWhHigIe7wNYskdmzeD0KAM9x2Gm7+c5rINbzut3j6G
        P7JzcNtWSi5hnqsvIzc38fE1NPu0+L5s2W8wvD9Yw6n44lwzValBIWCjZnCZaOfoYarUEEiuoaDPtT5o
        ytFqVzndOQqJSnEddhSeuLJUSXEdUDcvK7G1s5yU+pi2XPLnPn+ko/qPdq604fFjaLOTvGkk8uaAAIps
        w5wodEfu2yXMS6scI/mHjRfkT0kxbZbpnxD2nuyUFq24CPTfaB4guJpBsiCEJYtg7ELT4hq5GA785RWS
        wXIu8fHSnk1bgvLgjJ7mcQm1zI9nF+z1pCy2L2poG8SPGTxXQeSgNGabNIoZRasZASO63PHEILV5lkyS
        FdwVWedM9SwiQogFRwyjloHY9/nZFg4DwFXL1bu+0pGXoMf4oiB4Jjd72JjhZWNQQwJhOt6oqQwyXJoB
        qmNCO7nXyRSyg/svXE9RhnRdFZ9C0kEATbiApG581qR1A6xoUQrEV2BpBHFruhyPpuiC8OggF3b8bRCu
        WPVyGYgMLHAKdUaB5Thv68m5VOT9cQFS7ZsMXq/c4kptTw2Eoh5n6FkBwwK+P4jUHw40KDei1RhQjVg/
        xwTnlQuWrxmpgjFqpQY9rtw6mjgY4/k+R0chSdt4U8DddNRnaj5LxF7fvBiMcujNlt25xBM+yO4vIyZx
        1a+ADt0bf0='''
        raise urwid.ExitMainLoop()
    else:
        help_text.set_text(('help',
        '''Keys:
        q: quit,
        s: move sort column left,
        a: move sort column right,
        arrow keys: select/scroll lists
        No feedback on focus, sorry'''))
        loop.set_alarm_in(10, lambda l,y: help_text.set_text(''))

def update_messages(loop, tracker):
    del message_list[:]
    message_list.extend(tracker.get_message_stats())
    message_list._explicit_modified()
    loop.set_alarm_in(1,update_messages, tracker)

def update_connections(loop, data):
    socket_conns = get_connections()
    for conn_box in connection_pane_list:
        del conn_box.conn_list[:]
        conn_box.conn_list.extend(socket_conns[conn_box.socket_name])
    connection_pane_list[-1].conn_list.append(help_text)
    loop.set_alarm_in(5, update_connections, data)

palette = [
    ('no_sub',    'default',       'default'),
    ('sub_in',    'dark green',    'default'),
    ('sub_out',   'dark red',      'default'),
    ('sub_both',  'dark magenta',  'default'),
    ('val_in',    'dark green',    'default'),
    ('val_out',   'dark red',      'default'),
    ('bind',      'dark red',      'default'),
    ('connect',   'dark green',    'default'),
    ('help',      'light green',   'default', 'bold'),
]

loop = urwid.MainLoop(columns, palette, unhandled_input = handle_input)
loop.set_alarm_in(0.1, update_messages, stats_tracker)
loop.set_alarm_in(0.1, update_connections, None)
loop.run()
try: 
    import zlib
    import base64
    print(zlib.decompress(base64.b64decode(rd.replace('\n',''))))
except:
    pass
