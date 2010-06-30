#! /usr/bin/env python

import cauv
import cauv.node as node
import cauv.messaging as msg
import cauv.pipeline as pipeline

import time

n = node.Node("py-camtest")

pl = pipeline.Model(n)

pl.clear()
cam_forward_n = pl.addSynchronous(msg.NodeType.CameraInput)
pl.setParameterSynchronous(cam_forward_n, "device id", pipeline.intParam(1))

cam_down_n = pl.addSynchronous(msg.NodeType.CameraInput)
pl.setParameterSynchronous(cam_down_n, "device id", pipeline.intParam(0))

fo_forward_n = pl.addSynchronous(msg.NodeType.FileOutput)
pl.setParameterSynchronous(fo_forward_n, "filename", pipeline.stringParam("out.%d.%t.%c-forward.jpg"))

fo_down_n = pl.addSynchronous(msg.NodeType.FileOutput) 
pl.setParameterSynchronous(fo_forward_n, "filename", pipeline.stringParam("out.%d.%t.%c-down.jpg"))

pl.addArcSynchronous(cam_forward_n, "image_out", fo_forward_n, "image_in")
pl.addArcSynchronous(cam_down_n, "image_out", fo_down_n, "image_in")

