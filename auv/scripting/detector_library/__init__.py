#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import buoy_detector
import pipe_detector
import sonar_collision_detector
import test
import visual_collision_detector

index = [k for k,v in locals().items() if hasattr(v, "Detector")]