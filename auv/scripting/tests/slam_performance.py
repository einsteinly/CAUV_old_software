#! /usr/bin/env python
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import itertools
import math
import argparse
import csv
import datetime

# The SLAM poses are transformed so that they start from the same position and
# orientation as the first simulator pose.

def xyThetaTimeFromSimLog(sim_log_file):
    # return list of (x_in_metres, y_in_metres, theta_in_radians, time_in_seconds)
    r = []
    t0 = None
    for row in sim_log_file:
        displ_idx = row.find('displ=[')
        if displ_idx > 0:
            t_idx = row.find(':') - 2
            t_end = row.find(' ', t_idx)
            displ_idx = displ_idx + len('displ=[')
            displ_end = row.find(']', displ_idx)
            ori_idx = row.find('yaw = ') + len('yaw = ')
            ori_end = row.find(',', ori_idx)

            t_str = row[t_idx:t_end]
            displ_str = row[displ_idx:displ_end]
            x_str, y_str, z_str = displ_str.split(',')
            ori_str = row[ori_idx:ori_end]

            t = datetime.datetime.strptime(t_str, '%H:%M:%S.%f')
            if t0 is None:
                t0 = t
            t = (t - t0)
            t = t.seconds + t.microseconds / 1e6

            x = float(x_str)
            y = float(y_str)
            theta = math.radians(-float(ori_str))
            while theta > math.pi:
                theta = theta - 2*math.pi
            while theta <= -math.pi:
                theta = theta + 2*math.pi
            
            # x, y are rotated through 90 degrees compared to slam...
            r.append([y, -x, theta, t])
    return r

def xyThetaTimeFromPoseFile(slam_pose_file):
    # return list of (x_in_metres, y_in_metres, theta_in_radians, time_in_seconds)
    r = []
    sec0 = None
    musec0 = None
    for row in slam_pose_file:
        if row.startswith('pose'):
            (pose_literal, id, relto, x, y, theta, timestamp_literal, brace_literal,
             secs_literal, eq_literal, secs, musecs_literal, eq2_literal,
             musecs, brace2_literal, paren_literal) = row.split()
            assert(brace2_literal == '}' and timestamp_literal == 'TimeStamp')
            secs = secs.rstrip(',')
            if sec0 is None:
                sec0 = float(secs)
                musec0 = float(musecs)
            theta = float(theta)
            while theta > math.pi:
                theta = theta - 2*math.pi
            while theta <= -math.pi:
                theta = theta + 2*math.pi
            r.append([float(x), float(y), theta, float(secs)-sec0 + (float(musecs) - musec0) / 1e6])
    return r


def transformXYT(xytT, dx, dy, dtheta):
    # apply dx, dy FIRST, then rotate by dtheta about origin
    r = []
    for (x, y, t, T) in xytT:
        r.append([
            math.cos(dtheta) * (x + dx) - math.sin(dtheta) * (y + dy),
            math.sin(dtheta) * (x + dx) + math.cos(dtheta) * (y + dy),
            t + dtheta, 
            T
        ])
    return r

def writeCSV(rows, out_file):
    writer = csv.writer(out_file, dialect='excel') 
    writer.writerows(rows)

def processFiles(sim_log_file_name, slam_pose_file_name, out_file_name):
    with open(sim_log_file_name) as sim_log_file:
        sim_xytT  = xyThetaTimeFromSimLog(sim_log_file)
    with open(slam_pose_file_name) as slam_pose_file:
        slam_xytT = xyThetaTimeFromPoseFile(slam_pose_file)
    
    rows = [['sim t', 'sim x', 'sim y', 'sim theta', 'slam t', 'slam x', 'slam y', 'slam theta']]
    for simdata, slamdata in itertools.izip_longest(sim_xytT, slam_xytT):
        if slamdata is not None:
            (x, y, t, T) = slamdata
        if simdata is not None:
            (simx, simy, simt, simT) = simdata
        rows.append([simT, simx, simy, simt, T, x, y, t])
    
    with open(out_file_name, 'w') as out_file:
        writeCSV(rows, out_file)

def processPosesFile(slam_pose_file_name, out_file_name):
    with open(slam_pose_file_name) as slam_pose_file:
        slam_xytT = xyThetaTimeFromPoseFile(slam_pose_file)
    
    rows = [['slam t', 'slam x', 'slam y', 'slam theta']]
    for (x, y, t, T) in slam_xytT:
        rows.append([ T, x, y, t])

    with open(out_file_name, 'w') as out_file:
        writeCSV(rows, out_file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-log-file", '-s', dest='sim_log_file')
    parser.add_argument("--slam-pose-file", '-p', dest='slam_pose_file')
    parser.add_argument("out_file")

    opts  = parser.parse_args()

    processFiles(opts.sim_log_file, opts.slam_pose_file, opts.out_file)
    
    


