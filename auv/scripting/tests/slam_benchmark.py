#! /usr/bin/env python
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


# Perform a repeatable test

# Standard Library
import argparse
import datetime
import math
import time
import os

# CAUV
import cauv
import cauv.messaging as msg
import cauv.pipeline as pipeline
import cauv.control as control
import cauv.sonar as sonar
import cauv.node


class Benchmarker(object):
    def __init__(self, node, test_name):
        self.node = node
        self.pl = pipeline.Model(node) 
        self.auv = control.AUV(node)
        self.gemini = sonar.Gemini(node)
        # configuration:        
        self.test_name = test_name
        self.assoc_method = 'ICP' # 'ICP', 'NDT', 'non-linear ICP'        
        self.learn_keypoints = False
        self.visualisation_video = True
        #self.visualisation_files =         
        self.keypoints_video = False
        #self.viz_superzoom = 
        #self.viz_midzoom = 
        self.inter_ping_delay = 0.1
        self.resolution = 800
        # important parameters:
        self.reject_thr = 0.14     # higher = more support for score calculation, ransac support
        self.max_correspond = 0.75 # hard to explain
        self.weight_test = 0.5     # controls classifier ROC (0 = pass everything, 1 = fail almost everything)
        self.score_thr = 0.04      # max error permitted for match
        self.keyframe_spacing = 2.0# minimum distance between keyframes (sort of)
        self.max_matches = 3       # maximum number of pairwise correspondences attempted per scan
        self.required_consensus = 1# consensus required for match
        # internal stuff:
        self.video_output_nodes = []
        self.setup()

    def setup(self):
        self.auv.stop()
        self.auv.hbowMap(0,0)
        self.auv.hsternMap(0,0)
        self.auv.bearingParams(0.5, 0, 0, -1, 1, 1, 1, 1, 10) # much slower than normal!
        self.gemini.interPingDelay(6)
        self.gemini.range(20)
        self.gemini.gain(60)
        self.gemini.rangeLines(self.resolution)
        self.gemini.continuous(True)
        self.loadPipeline()
        # give the GUI time to get sorted out
        time.sleep(4)
        # now set the data flowing:
        self.gemini.interPingDelay(self.inter_ping_delay)
        # make sure slam inits at the sim origin
        time.sleep(4)

    def teardown(self):
        # make sure videos get written:
        for node in self.video_output_nodes:
            node.x()
        self.video_output_nodes = []
        # close this to dump classifier stats to the log
        self.sslam.x()

    def loadPipeline(self):
        nt = msg.NodeType
        pl = self.pl
        pl.clear()
        sonar_in = pl.addNode(nt.SonarInput)
        sonar_in.p('Sonar ID').set(2)
        sonar_in.p('Resolution').set(self.resolution)
        resize = pl.addNode(nt.Resize)
        resize.p('fixed width').set(256)
        resize.p('fixed height').set(self.resolution)
        sonar_in.o('polar image').connect(resize.i('image_in'))
        #copy1 = pl.addNode(nt.Copy)
        #guio = pl.addNode(nt.GuiOutput)
        #sonar_in.o('polar image').connect(copy1.i('image'))
        #sonar_in.o('image (synced)').connect(guio.i('image_in'))
        medf = pl.addNode(nt.MedianFilter)
        medf.p('kernel').set(1)
        resize.o('image_out').connect(medf.i('image'))        
        copy2 = pl.addNode(nt.Copy)
        medf.o('image (not copied)').connect(copy2.i('image'))
        blur1 = pl.addNode(nt.GaussianBlur)
        blur1.p('sigma').set(80.0)
        copy2.o('image copy').connect(blur1.i('image'))
        mix1  = pl.addNode(nt.Mix)
        mix1.p('absolute value').set(False)
        mix1.p('image fac').set(-6.0)
        mix1.p('mix fac').set(2.0)
        medf.o('image (not copied)').connect(mix1.i('mix'))
        blur1.o('image (not copied)').connect(mix1.i('image'))
        copy3 = pl.addNode(nt.Copy)
        mix1.o('image (not copied)').connect(copy3.i('image'))
        levels = pl.addNode(nt.Levels)
        levels.p('black level').set(msg.Colour.fromGrey(8))
        levels.p('white level').set(msg.Colour.fromGrey(255))
        #ssf = pl.addNode(nt.SonarShadowFilter)
        #ssf.p('object importance').set(1.0)
        #ssf.p('object size').set(0.2)
        #ssf.p('shadow importance').set(1.0)
        #ssf.p('shadow size').set(20.0)
        #copy3.o('image copy').connect(ssf.i('polar image'))
        #ssf.o('polar image').connect(levels.i('image'))
        copy3.o('image copy').connect(levels.i('image'))
        nop = pl.addNode(nt.Nop)
        levels.o('image (not copied)').connect(nop.i('image in'))
        corners = pl.addNode(nt.FASTCorners)
        corners.p('non-maximum suppression').set(True)
        corners.p('threshold').set(40)
        br_crop = pl.addNode(nt.BearingRangeCrop)
        br_crop.p('bearing start').set(-50.0)
        br_crop.p('bearing end').set(50.0)
        br_crop.p('range start').set(0.0)
        br_crop.p('range end').set(120.0)
        delay = pl.addNode(nt.Delay)
        delay.p('delay (frames)').set(1)
        br_to_xy = pl.addNode(nt.BearingRangeToXY)
        draw_kps = pl.addNode(nt.DrawKeyPoints)
        correl = pl.addNode(nt.Correlation1D)
        sslam = pl.addNode(nt.SonarSLAM)
        self.sslam = sslam
        sslam.p('-vis origin x').set(-7.0)
        sslam.p('-vis origin y').set(-7.0)
        sslam.p('-vis resolution').set(400)
        sslam.p('-vis size').set(36.0)
        sslam.p('clear').set(False)
        sslam.p('euclidean fitness').set(1e-7)
        sslam.p('feature merge distance').set(0.2)
        sslam.p('graph iters').set(10)
        sslam.p('max matches').set(int(self.max_matches))
        sslam.p('match consensus').set(int(self.required_consensus))        
        sslam.p('keyframe spacing').set(self.keyframe_spacing)
        assert(self.assoc_method in ('ICP', 'NDT', 'non-linear ICP'))
        sslam.p('match algorithm').set(self.assoc_method)
        sslam.p('grid step').set(3.5)
        sslam.p('ransac iterations').set(0)
        # !!! TODO: maybe increase this some more...
        sslam.p('max correspond dist').set(float(self.max_correspond)) 
        sslam.p('max iters').set(20)
        sslam.p('overlap threshold').set(0.3)
        sslam.p('reject threshold').set(float(self.reject_thr))
        sslam.p('score threshold').set(float(self.score_thr))
        sslam.p('transform eps').set(1e-9)
        sslam.p('weight test').set(float(self.weight_test))
        sslam.p('xy metres/px').set(0.01) # unused, anyway
        nop.o('image out (not copied)').connect(corners.i('image in'))
        nop.o('image out (not copied)').connect(br_crop.i('polar image'))
        nop.o('image out (not copied)').connect(draw_kps.i('image in'))
        nop.o('image out (not copied)').connect(sslam.i('keypoints image'))
        if self.learn_keypoints:
            learn_kps = pl.addNode(nt.LearnedKeyPoints)
            learn_kps.p("questions").set(400)
            learn_kps.p("trees").set(50)
            nop.o('image out (not copied)').connect(learn_kps.i('image'))
            corners.o('keypoints').connect(learn_kps.i('bootstrap keypoints'))
            learn_kps.o('good keypoints').connect(sslam.i('training: polar keypoints'))
            learn_kps.o('good keypoints').connect(draw_kps.i('KeyPoints'))
            learn_kps.o('good keypoints').connect(br_to_xy.i('keypoints'))
            learn_kps.o('image').connect(br_to_xy.i('polar image'))
            learn_kps.o('image').connect(sslam.i('keypoints image'))
            sslam.o('training: goodness').connect(learn_kps.i('training: goodness'))
            sslam.o('training: keypoints').connect(learn_kps.i('training: keypoints'))
            sslam.o('training: keypoints image').connect(learn_kps.i('training: keypoints image'))
        else:
            nop.o('image out (not copied)').connect(br_to_xy.i('polar image'))
            corners.o('keypoints').connect(br_to_xy.i('keypoints'))
            corners.o('keypoints').connect(sslam.i('training: polar keypoints'))
            corners.o('keypoints').connect(draw_kps.i('KeyPoints'))
        br_to_xy.o('keypoints').connect(sslam.i('keypoints'))
        br_crop.o('polar image').connect(correl.i('Image A'))
        br_crop.o('polar image').connect(delay.i('image in'))
        delay.o('image out (not copied)').connect(correl.i('Image B'))
        clamp = pl.addNode(nt.ClampFloat)
        clamp.p('Max').set(0.2)
        clamp.p('Min').set(-0.2)
        correl.o('max correl location').connect(clamp.i('Value'))
        clamp.o('Value').connect(sslam.i('delta theta'))
        resize = pl.addNode(nt.Resize)
        draw_kps.o('image out').connect(resize.i('image_in'))
        rotate = pl.addNode(nt.Rotate)
        rotate.p('extend').set(True)
        rotate.p('radians').set(math.pi/2)
        resize.o('image_out').connect(rotate.i('image_in'))
        if self.keypoints_video:
            kps_video_out = pl.addNode(nt.VideoFileOutput)
            kps_video_out.param('filename').set('%s-keypoints.avi' % self.test_name)
            rotate.o('image_out').connect(kps_video_out.i('image'))
            self.video_output_nodes.append(kps_video_out)
        copyviz = pl.addNode(nt.Copy)
        sslam.o('cloud visualisation').connect(copyviz.i('image'))
        if self.visualisation_video:
            viz_video_out = pl.addNode(nt.VideoFileOutput)
            viz_video_out.param('filename').set('%s-map.avi' % self.test_name)
            copyviz.o('image copy').connect(viz_video_out.i('image'))
            self.video_output_nodes.append(viz_video_out)
        guio = pl.addNode(nt.GuiOutput)
        copyviz.o('image copy').connect(guio.i('image_in'))

    def runTest(self):
        #self.runTest_spin()
        self.runTest_loop()
        #self.runTest_short()
        #self.runTest_reverseLoop()
    
    def runTest_spin(self):
        for bearing in(0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360):
            self.auv.bearingAndWait(-bearing)
    
    def runTest_short(self):
        self.auv.bearingAndWait(6)
        self.auv.bearingAndWait(0)
        self.auv.prop(110)
        time.sleep(57)
        self.auv.stop()

    def runTest_reverseLoop(self):
        self.auv.bearingAndWait(-6)

        for bearing in (90, 0, -90, -180):
            self.auv.bearingAndWait(bearing)
            self.auv.prop(120)
            time.sleep(57)
            self.auv.prop(0)
            time.sleep(1)

        self.auv.stop()


    def runTest_loop(self):
        self.auv.bearingAndWait(-6)
        self.auv.bearingAndWait(0)
        self.auv.prop(110)
        #time.sleep(57)

        time.sleep(11.4)
        for x in [1,2]:
            self.auv.bearing(15)
            self.auv.prop(110)
            time.sleep(11.4)
            self.auv.bearing(-15)
            self.auv.prop(110)
            time.sleep(11.4)

        self.auv.prop(0)
        self.auv.bearingAndWait(45)
        self.auv.prop(110)
        time.sleep(10)
        self.auv.prop(0) 

        self.auv.bearingAndWait(90)
        self.auv.prop(110)
        time.sleep(48)
        self.auv.prop(0)
        self.auv.bearingAndWait(135)
        self.auv.prop(110)
        time.sleep(10)
        self.auv.prop(0) 

        self.auv.bearingAndWait(180)
        self.auv.prop(110)
        time.sleep(48)
        self.auv.prop(0)
        self.auv.bearingAndWait(225)
        self.auv.prop(110)
        time.sleep(10)
        self.auv.prop(0) 

        self.auv.bearingAndWait(270)
        self.auv.prop(110)
        time.sleep(48)
        self.auv.prop(0)
        self.auv.bearingAndWait(315)
        self.auv.prop(110)
        time.sleep(10)
        self.auv.prop(0) 

        self.auv.bearingAndWait(0)

        self.auv.stop()

        time.sleep(10)
    
    def processResults(self):
        from tools import slam_performance
        import os
        
        # find the latest slam pose dump:
        dirs = [os.path.join('/tmp/cauv/slamdump/',d) for d in os.listdir('/tmp/cauv/slamdump/')]
        latest_dir = max(dirs, key=os.path.getmtime)
        # and latest pose file in directory:
        files = [os.path.join(latest_dir, f) for f in os.listdir(latest_dir) if f.startswith('poses')]
        poses_file = max(files, key=os.path.getmtime)

        # find the latest simulator log file
        files = [os.path.join('./session-logs/', d) for d in os.listdir('./session-logs/') if d.find('sim.py.log') != -1]
        sim_logfile = max(files, key=os.path.getmtime)

        out_file = '%s-%s-learn=%s.csv' % (
            self.test_name,
            self.assoc_method,
            int(self.learn_keypoints)
        )

        slam_performance.processFiles(sim_logfile, poses_file, out_file)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs='?', default=datetime.datetime.utcnow().strftime('slam-benchmark-%Y%m%d%H%M%S'))

    opts, unknown_args = parser.parse_known_args()
    node = cauv.node.Node('py-slamb', unknown_args)

    try:
        b = Benchmarker(node, opts.name)
        b.runTest()
        b.teardown()
        b.processResults()
    finally:
        node.stop()
