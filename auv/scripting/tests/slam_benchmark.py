#! /usr/bin/env python

# Perform a repeatable test

# Standard Library
import argparse
import datetime
import math
import time

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
        self.test_name = test_name
       
        self.video_output_nodes = []

        self.setup()

    def setup(self):
        self.auv.stop()
        self.auv.hbowMap(0,0)
        self.auv.hsternMap(0,0)
        self.auv.bearingParams(1, 0, 0, -1, 1, 1, 1, 1, 10) # much slower than normal!
        self.gemini.interPingDelay(6)
        self.gemini.range(20)
        self.gemini.gain(60)
        self.gemini.rangeLines(400)
        self.gemini.continuous(True)
        self.loadPipeline()
        # give the GUI time to get sorted out
        time.sleep(3)
        # now set the data flowing:
        self.gemini.interPingDelay(0.2)        

    def teardown(self):
        # make sure videos get written:
        for node in self.video_output_nodes:
            node.x()
        self.video_output_nodes = []

    def loadPipeline(self):
        nt = msg.NodeType
        pl = self.pl
        pl.clear()
        sonar_in = pl.addNode(nt.SonarInput)
        sonar_in.p('Sonar ID').set(2)
        sonar_in.p('Resolution').set(800)
        copy1 = pl.addNode(nt.Copy)
        guio = pl.addNode(nt.GuiOutput)
        sonar_in.o('polar image').connect(copy1.i('image'))
        sonar_in.o('image (synced)').connect(guio.i('image_in'))
        medf = pl.addNode(nt.MedianFilter)
        medf.p('kernel').set(1)
        copy1.o('image copy').connect(medf.i('image'))
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
        levels.p('black level').set(msg.BoundedFloat(8, 0, 255, msg.BoundedFloatType.Clamps))
        levels.p('white level').set(msg.BoundedFloat(255, 0, 255, msg.BoundedFloatType.Clamps))
        copy3.o('image copy').connect(levels.i('image'))
        nop = pl.addNode(nt.Nop)
        levels.o('image (not copied)').connect(nop.i('image in'))
        corners = pl.addNode(nt.FASTCorners)
        corners.p('non-maximum suppression').set(True)
        corners.p('threshold').set(40)
        delay = pl.addNode(nt.Delay)
        delay.p('delay (frames)').set(1)
        br_to_xy = pl.addNode(nt.BearingRangeToXY)
        draw_kps = pl.addNode(nt.DrawKeyPoints)
        correl = pl.addNode(nt.Correlation1D)
        sslam = pl.addNode(nt.SonarSLAM)
        sslam.p('-vis origin x').set(-4.0)
        sslam.p('-vis origin y').set(-8.0)
        sslam.p('-vis resolution').set(800)
        sslam.p('-vis size').set(36.0)
        sslam.p('clear').set(False)
        sslam.p('euclidean fitness').set(1e-7)
        sslam.p('feature merge distance').set(0.1)
        sslam.p('graph iters').set(10)
        sslam.p('keyframe spacing').set(1.5)
        sslam.p('match algorithm').set('ICP')
        # !!! TODO: sensitivity to this:
        sslam.p('max correspond dist').set(0.5)
        sslam.p('max iters').set(80)
        sslam.p('overlap threshold').set(0.5)
        sslam.p('reject threshold').set(0.2)
        sslam.p('score threshold').set(0.04)
        sslam.p('transform eps').set(1e-9)
        sslam.p('weight test').set(-1.0)
        sslam.p('xy metres/px').set(0.01) # unused, anyway
        nop.o('image out (not copied)').connect(corners.i('image in'))
        nop.o('image out (not copied)').connect(delay.i('image in'))
        nop.o('image out (not copied)').connect(br_to_xy.i('polar image'))
        nop.o('image out (not copied)').connect(draw_kps.i('image in'))
        nop.o('image out (not copied)').connect(correl.i('Image A'))
        nop.o('image out (not copied)').connect(sslam.i('keypoints image'))
        corners.o('keypoints').connect(draw_kps.i('KeyPoints'))
        corners.o('keypoints').connect(br_to_xy.i('keypoints'))
        corners.o('keypoints').connect(sslam.i('training: polar keypoints'))
        br_to_xy.o('keypoints').connect(sslam.i('keypoints'))
        delay.o('image out (not copied)').connect(correl.i('Image B'))
        clamp = pl.addNode(nt.ClampFloat)
        clamp.p('Max').set(0.1)
        clamp.p('Min').set(-0.1)
        correl.o('max correl location').connect(clamp.i('Value'))
        clamp.o('Value').connect(sslam.i('delta theta'))
        resize = pl.addNode(nt.Resize)
        draw_kps.o('image out').connect(resize.i('image_in'))
        rotate = pl.addNode(nt.Rotate)
        rotate.p('extend').set(True)
        rotate.p('radians').set(math.pi/2)
        resize.o('image_out').connect(rotate.i('image_in'))
        kps_video_out = pl.addNode(nt.VideoFileOutput)
        kps_video_out.param('filename').set('%s-keypoints.avi' % self.test_name)
        rotate.o('image_out').connect(kps_video_out.i('image'))
        copyviz = pl.addNode(nt.Copy)
        sslam.o('cloud visualisation').connect(copyviz.i('image'))
        viz_video_out = pl.addNode(nt.VideoFileOutput)
        viz_video_out.param('filename').set('%s-map.avi' % self.test_name)
        guio = pl.addNode(nt.GuiOutput)
        copyviz.o('image copy').connect(viz_video_out.i('image'))
        copyviz.o('image copy').connect(guio.i('image_in'))

        self.video_output_nodes.append(kps_video_out)
        self.video_output_nodes.append(viz_video_out)

    def runTest(self):
        self.auv.bearingAndWait(6)
        self.auv.bearingAndWait(0)
        self.auv.prop(110)
        time.sleep(60)
        self.auv.prop(0)

        self.auv.bearingAndWait(90)
        self.auv.prop(110)
        time.sleep(60)
        self.auv.prop(0)

        self.auv.bearingAndWait(180)
        self.auv.prop(110)
        time.sleep(60)
        self.auv.prop(0)

        self.auv.bearingAndWait(270)
        self.auv.prop(110)
        time.sleep(60)
        self.auv.prop(0)

        self.auv.bearingAndWait(0)

        self.auv.stop()

        time.sleep(10)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs='?', default=datetime.datetime.utcnow().strftime('slam-benchmark-%Y%m%d%H%M%S'))

    opts, unknown_args = parser.parse_known_args()
    node = cauv.node.Node('py-slamb', unknown_args)

    try:
        b = Benchmarker(node, opts.name)
        b.runTest()
        b.teardown()
    finally:
        node.stop()
