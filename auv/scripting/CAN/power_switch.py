#!/usr/bin/env python2.7
import struct
import binascii
import argparse
dev_map = {
    "panda" : 0,
    "seco" : 1,
    "can_aft" : 2,
    "can_fore" : 3,
    "gemini" : 4,
    "antenna" : 5,
    "imu" : 6,
    "enet" : 7,
    "seasprite" : 8,
    "spd_ctrl" : 9
}
parser = argparse.ArgumentParser(description="switch PDB batteries")
parser.add_argument('state', choices = ['off', 'on'])
parser.add_argument('device', help='left or right battery', choices = dev_map.keys())
args = parser.parse_args()

s = struct.pack('<LBBBxxxxxx',9,2,dev_map[args.device],
                 {'on': 1, 'off': 0}[args.state])
print('\xc0\x1d\xbe\xef' + s)
