#!/usr/bin/env python2.7
import struct
import binascii
import argparse
parser = argparse.ArgumentParser(description="switch PDB batteries")
parser.add_argument('-p','--position', default='left', help='left or right battery')
args = parser.parse_args()
if args.position == 'left':
    val = 0
else:
    val = 1
s = struct.pack('<LBBxxxxxxx',8,1,val)
print('\xc0\x1d\xbe\xef' + s)
