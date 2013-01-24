#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import json
import re
import argparse
import collections
import struct
import serial

delimiter = '\xc0\x1d\xbe\xef'

parser = argparse.ArgumentParser(description = 
    "Generate CAN frame related structs and functions")
parser.add_argument("--msg-file", help="message file to parse", default="frames.json")
parser.add_argument("--listen", help="listen to CAN messages on this port")
parser.add_argument("--send", help="send json-encoded CAN message")
opts = parser.parse_args()


with open(opts.msg_file) as json_file:
    #comments!
    json_string = re.sub("//[^\n]+\n","\n", json_file.read())
    frames = json.loads(json_string, object_pairs_hook = collections.OrderedDict)

struct_map = collections.defaultdict(lambda : None)
struct_map.update ({
    "uint8_t" : "B",
    "int8_t" : "b",
    "uint16_t" : "H",
    "int16_t" : "h",
    "uint32_t" : "L",
    "int32_t" : "l",
    "float" : "f",
    "double" : "d",
})

msg_map = {}

for msg_name, msg in frames.items():
    members_map = []
    for m_name, m_type in msg['members'].items():
        try:
            members_map.append((m_name, struct_map[m_type]))
        except TypeError:
            members_map.append((m_name, "B"))
    if (members_map):
        member_names, member_structs = zip(*members_map)
        print('{}: {}, {}'.format(msg_name, ' '.join(member_names), ''.join(member_structs)))

    msg_map[msg['id']] = (msg_name, members_map)

if opts.listen:
    s = serial.Serial(opts.listen, 115200)
    while True:
        sync_pos = 0
        while sync_pos < len(delimiter):
            c = s.read(1)
            if delimiter[sync_pos] == c:
                sync_pos += 1
            else:
                sync_pos = 0
        frame_fmt = "<LB8s"
        msg_id, length, m_bytes = struct.unpack(frame_fmt, s.read(struct.calcsize(frame_fmt)))
        msg_info = msg_map[msg_id]
        msg_fmt = ''.join(zip(*msg_info[1])[1])
        members = struct.unpack("<" + msg_fmt, m_bytes[:length])
        print(msg_info[0] + ":  " + ' '.join(["{}: {}".format(*t) for t in zip(zip(*msg_info[1])[0], members)]))
