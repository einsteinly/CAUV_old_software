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
        # configuration:
        self.test_name = test_name
        self.assoc_method = 'NDT' # 'ICP', 'NDT', 'non-linear ICP'
        self.learn_keypoints = False
        self.visualisation_video = False
        self.visualisation_files = False
        self.keypoints_video = False
        self.viz_superzoom = False
        self.resolution = 800
        # internal stuff:
        self.video_output_nodes = []
        self.setup()

    def setup(self):
        self.loadPipeline()
        # give the GUI time to get sorted out
        time.sleep(2.0)

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
        sonar_in.p('Resolution').set(self.resolution)
        resize = pl.addNode(nt.Resize)
        resize.p('fixed width').set(256)
        resize.p('fixed height').set(self.resolution)
        sonar_in.o('polar image').connect(resize.i('image_in'))
        #guio = pl.addNode(nt.GuiOutput)
        #sonar_in.o('image (synced)').connect(guio.i('image_in'))
        # This levels node improves the conditioning when we high-pass filter
        levels0 = pl.addNode(nt.Levels)
        levels0.p('black level').set(msg.BoundedFloat(  0, 0, 255, msg.BoundedFloatType.Clamps))
        levels0.p('white level').set(msg.BoundedFloat( 64, 0, 255, msg.BoundedFloatType.Clamps))
        resize.o('image_out').connect(levels0.i('image'))
        medf = pl.addNode(nt.MedianFilter)
        medf.p('kernel').set(1) # TODO: test performance as this varies
        levels0.o('image (not copied)').connect(medf.i('image'))
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
        #levels = pl.addNode(nt.Levels)
        #levels.p('black level').set(msg.BoundedFloat(  0, 0, 255, msg.BoundedFloatType.Clamps))
        #levels.p('white level').set(msg.BoundedFloat(255, 0, 255, msg.BoundedFloatType.Clamps))
        #copy3.o('image copy').connect(levels.i('image'))
        ssf = pl.addNode(nt.SonarShadowFilter)
        ssf.p('object importance').set(1.0)
        ssf.p('object size').set(0.2)
        ssf.p('shadow importance').set(1.0)
        ssf.p('shadow size').set(5.0)
        copy3.o('image copy').connect(ssf.i('polar image'))
        nop = pl.addNode(nt.Nop)
        #levels.o('image (not copied)').connect(nop.i('image in'))
        ssf.o('polar image').connect(nop.i('image in'))
        corners = pl.addNode(nt.FASTCorners)
        corners.p('non-maximum suppression').set(True)
        corners.p('threshold').set(30)
        br_crop = pl.addNode(nt.BearingRangeCrop)
        br_crop.p('bearing start').set(-20.0)
        br_crop.p('bearing end').set(20.0)
        br_crop.p('range start').set(0.0)
        br_crop.p('range end').set(120.0)
        delay = pl.addNode(nt.Delay)
        delay.p('delay (frames)').set(1)
        br_to_xy = pl.addNode(nt.BearingRangeToXY)
        draw_kps = pl.addNode(nt.DrawKeyPoints)
        correl = pl.addNode(nt.Correlation1D)
        sslam = pl.addNode(nt.SonarSLAM)
        if self.viz_superzoom:
            sslam.p('-vis origin x').set(-1.0)
            sslam.p('-vis origin y').set(-23.0)
            sslam.p('-vis resolution').set(1200)
            sslam.p('-vis size').set(40.0)
        else:
            sslam.p('-vis origin x').set(-2.0)
            sslam.p('-vis origin y').set(-50.0)
            sslam.p('-vis resolution').set(1200)
            sslam.p('-vis size').set(100.0)
        sslam.p('clear').set(False)
        sslam.p('euclidean fitness').set(1e-7)
        sslam.p('feature merge distance').set(0.2)
        sslam.p('graph iters').set(10)
        #sslam.p('keyframe spacing').set(1.0)
        sslam.p('keyframe spacing').set(4.0) # changed for viz demo
        assert(self.assoc_method in ('ICP', 'NDT', 'non-linear ICP'))
        sslam.p('match algorithm').set(self.assoc_method)
        sslam.p('grid step').set(2.5)
        sslam.p('ransac iterations').set(10)
        # !!! TODO: sensitivity to this:
        sslam.p('max correspond dist').set(0.35)
        sslam.p('max iters').set(20)
        sslam.p('overlap threshold').set(0.3)
        sslam.p('reject threshold').set(0.1)
        sslam.p('score threshold').set(0.08)
        sslam.p('transform eps').set(1e-9)
        sslam.p('weight test').set(0.5)
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
        crop = pl.addNode(nt.Crop)
        crop.p('height').set(600)
        crop.p('width').set(1200)
        crop.p('top left (x)').set(0)
        crop.p('top left (y)').set(300)
        copyviz.o('image copy').connect(crop.i('image in'))
        if self.visualisation_video:
            viz_video_out = pl.addNode(nt.VideoFileOutput)
            viz_video_out.param('filename').set('%s-map.avi' % self.test_name)
            crop.o('image out (not copied)').connect(viz_video_out.i('image'))
            self.video_output_nodes.append(viz_video_out)
        if self.visualisation_files:
            split = pl.addNode(nt.SplitHSV)
            crop.o('image out (not copied)').connect(split.i('image'))
            invert = pl.addNode(nt.Invert)
            mixv = pl.addNode(nt.MixValue)
            mixv.p('ch1').set(100)
            combine = pl.addNode(nt.CombineHSV)
            split.o('H').connect(combine.i('H'))
            split.o('S').connect(combine.i('S'))
            split.o('V').connect(invert.i('image'))
            invert.o('image (not copied)').connect(mixv.i('image'))
            mixv.o('image (not copied)').connect(combine.i('V'))
            viz_files_out = pl.addNode(nt.FileOutput)
            viz_files_out.param('filename').set('/tmp/viz/%s-map-%%c.jpg' % self.test_name)
            combine.o('image').connect(viz_files_out.i('image_in'))
        guio = pl.addNode(nt.GuiOutput)
        crop.o('image out (not copied)').connect(guio.i('image_in'))
    def runTest(self):
        #time.sleep(60.0*60.0*2.0)
        time.sleep(30)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs='?', default=datetime.datetime.utcnow().strftime('slam-realdata-%Y%m%d%H%M%S'))

    opts, unknown_args = parser.parse_known_args()
    node = cauv.node.Node('py-slamr', unknown_args)

    try:
        b = Benchmarker(node, opts.name)
        b.runTest()
        b.teardown()
    finally:
        node.stop()

