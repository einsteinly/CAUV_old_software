#!/usr/bin/env python2.7
from cauv.control_extended import AUV
from cauv.node import Node

node = Node("test")
auv = AUV(node)
auv.moveToRelativeLocationAndWait((10,-40))