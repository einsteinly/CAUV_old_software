#!/usr/bin/env python2.7
import struct
import binascii
import argparse
parser = argparse.ArgumentParser(description="switch PDB batteries")
parser.add_argument('-d','--device', help='left or right battery')
parser.add_argument('--on', action='store_true')
parser.add_argument('--off', action='store_true')
args = parser.parse_args()

if args.on and args.off:
    print('fuck off!')
    exit(1)

dev_map = {
    'gemini' : 4,
    'can_back' : 2,
    'panda' : 0
}
if args.on:
    val = 1
elif args.off:
    val = 0
s = struct.pack('<LBBBxxxxxx',9,2,dev_map[args.device],val)
print('\xc0\x1d\xbe\xef' + s)
