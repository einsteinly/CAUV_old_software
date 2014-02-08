#!/usr/bin/env pypy

import argparse
from cauv_slam.msg import SonarImage
import genpy
import os.path
import re

parser = argparse.ArgumentParser(description="Parse CHIL logs for sonar data")
parser.add_argument("log_file", help='log file to parse')
parser.add_argument("--output_dir", help='directory to output data to')

args = parser.parse_args()

#struct PolarImage
#{
#    data: list<byte>;
#    // currently only raw uint8_t (isodistant responses in fastest varying index) is supported
#    encoding: ImageEncodingType;
#    // each bearing in range (-6400 to 6400) * 0x10000, these are the edges of
#    // the bins that each iso-bearing beam of data falls into, so there are
#    // num_beams + 1 values. The first bin is bearing_bins[0] --
#    // bearing_bins[1], and so on.
#    // Note that these are *bearings* not angles (ie, clockwise from forwards, not
#    // anticlockwise from x axis (wherever that is...))
#    bearing_bins: list<int32>;
#    // number of bins = round((rangeEnd-rangeStart) / rangeConversion)
#    rangeStart: float;
#    rangeEnd: float;
#    // rangeConversion = length of 1 bin in metres
#    rangeConversion: float;
#    // time of the ping from which the image is acquired
#    timeStamp : TimeStamp;
#}

#Wheeee regex
parse_re = re.compile("""^

(?P<time>\d+)\ 
(?P<msg_id>\d+)
\(\d+,
   \(
   (?P<image>[0-9A-F]+),
   \d+, #encoding
   \((?P<bearing_bins>(-?\d+,)+(-?\d+))\),
   (?P<rangeStart>-?\d+\.?\d*),
   (?P<rangeEnd>-?\d+\.?\d*),
   (?P<rangeConversion>-?\d+\.?\d*),
   \(
        (?P<time_secs>-?\d+),
        (?P<time_usecs>-?\d+)
   \)
   \)
\)
$"""
        , re.VERBOSE)

n_images = 0
for line in open(args.log_file):
    match = parse_re.match(line)
    if match is None:
        continue
    match = match.groupdict()
    n_images += 1
    image_hex = match['image']
    image_string = "".join([chr(int(image_hex[i*2:i*2+2], 16)) for i in range(len(image_hex) / 2)])
    bearings = [int(x) for x in match['bearing_bins'].split(",")]
    rangeStart = float(match['rangeStart'])
    rangeEnd = float(match['rangeEnd'])
    rangeConversion = float(match['rangeConversion'])
    time_secs = int(match['time_secs'])
    time_usecs = int(match['time_usecs'])
    msg = SonarImage(image_string, bearings, rangeStart, rangeEnd,
            rangeConversion, genpy.Time(time_secs, time_usecs * 1000)) 
    if args.output_dir:
        output_file_name = os.path.join(args.output_dir, "SonarImage{:0>6}.rosmsg".format(n_images))
        print(output_file_name)
        with open(output_file_name, 'wb') as output_file:
            msg.serialize(output_file)
    print(n_images)
