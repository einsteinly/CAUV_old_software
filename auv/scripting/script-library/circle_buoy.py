#!/usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.control as control
import cauv.pipeline as pipeline
import cauv.node
from cauv.debug import debug, info, warning, error

import time
import optparse

Do_Turn_Limit = 100
Do_Prop_Limit = 50
Circles_Position_Pixel_Width = 300

class BouyCircleObserver(msg.MessageObserver):
    def __init__(self, cauv_node, auv, strafe_speed, buoy_size,
                 pos_kpid = (1,0,0), size_kpid=(1,0,0)):
        msg.MessageObserver.__init__(self)
        self.__node = cauv_node
        self.__node.join('processing')
        self.__node.addObserver(self)
        self.__auv = auv
        self.__pl = pipeline.Model(self.__node)
        self.__strafe_speed = strafe_speed
        self.__current_turn_speed = 0
        self.__buoy_size = buoy_size
        self.last_pos_err = None
        self.sum_pos_err = 0
        self.last_size_err = None
        self.sum_size_err = 0
        self.__poskpid = pos_kpid
        self.__sizekpid = size_kpid
    
    def loadPipeline(self):
        self.__pl.load('pipelines/circle_buoy.pipe')

    def onCirclesMessage(self, m):
        if str(m.name) == 'buoy':
            debug('Received circles message: %s' % str(m))
            mean_circle_position = 0.5
            mean_circle_radius = 1
            num_circles = len(m.circles)
            if num_circles != 0:
                mean_circle_position = 0
                mean_circle_radius = 0
                for circle in m.circles:
                    mean_circle_position += circle.centre.x
                    mean_circle_radius += circle.radius
                mean_circle_position /= num_circles
                mean_circle_radius /= num_circles
                debug('buoy at %s %s' % (mean_circle_position,
                                         mean_circle_radius))
                self.actOnBuoy(mean_circle_position,
                               mean_circle_radius)
            else:
                debug('no circles!')
        else:
            debug('Ignoring circles message: %s' % str(m))

    def actOnBuoy(self, centre, radius):
        pos_err = centre - 0.5
        pos_derr = 0
        self.sum_pos_err += pos_err
        pos_ierr = self.sum_pos_err
        now = time.time()
        if self.last_pos_err is not None:
            pos_derr = (pos_err - self.last_pos_err[1]) / (now - self.last_pos_err[0])
        self.last_pos_err = (now, pos_err)
        do_turn = self.__poskpid[0] * pos_err +\
                  self.__poskpid[1] * pos_derr +\
                  self.__poskpid[2] * pos_ierr
        if do_turn > Do_Turn_Limit:
            do_turn = Do_Turn_Limit
        if do_turn < -Do_Turn_Limit:
            do_turn = -Do_Turn_Limit
        debug('turning: %s' % do_turn)
        self.__current_turn_speed = do_turn
        self.updateMotors()

        size_err = radius - self.__buoy_size
        size_derr = 0
        self.sum_size_err += size_err
        size_ierr = self.sum_size_err
        if self.last_size_err is not None:
            size_derr = (size_err - self.last_size_err[1]) / (now - self.last_size_err[0])
        self.last_size_err = (now, size_err)
        do_prop = self.__sizekpid[0] * size_err +\
                  self.__sizekpid[1] * size_derr +\
                  self.__sizekpid[2] * size_ierr
        if do_prop > Do_Prop_Limit:
            do_prop = Do_Prop_Limit
        if do_prop < -Do_Prop_Limit:
            do_prop = -Do_Prop_Limit
        debug('setting prop: %s' % do_prop)
        self.__auv.prop(int(round(do_prop)))
    
    def updateMotors(self):
        self.__auv.hbow(int(round(self.__strafe_speed + self.__current_turn_speed)))
        self.__auv.hbow(int(round(self.__strafe_speed - self.__current_turn_speed)))

    def run(self):
        self.loadPipeline()
        self.__auv.stop()
        start_bearing = self.__auv.getBearing()
        entered_quarters = [False, False, False, False]
        info('waiting for circles....')
        self.updateMotors()
        try:
            while False in entered_quarters:
                time.sleep(0.5)
                if self.__auv.getBearing() > -180 and self.__auv.getBearing() < -90:
                    entered_quarters[3] = True
                if self.__auv.getBearing() > -90 and self.__auv.getBearing() < 0:
                    entered_quarters[2] = True
                if self.__auv.getBearing() > 0 and self.__auv.getBearing() < 90:
                    entered_quarters[0] = True
                if self.__auv.getBearing() > 90 and self.__auv.getBearing() < 180:
                    entered_quarters[1] = True
                if self.__auv.getBearing() > 180 and self.__auv.getBearing() < 270:
                    entered_quarters[2] = True
                if self.__auv.getBearing() > 270 and self.__auv.getBearing() < 360:
                    entered_quarters[3] = True
            while self.__auv.getBearing() < start_bearing:
                info('waiting for final completion...')
                time.sleep(0.5)
        finally:
            info('stopping...')
            self.__auv.stop()
        info('complete!')
            


def runWithNode(cauv_node, auv, opts=None):
    # opts.strafe_speed (int [-127,127]) controls strafe speed
    # opts.buoy_size (float [0.0, 1.0]) controls distance from buoy. Units are
    # field of view that the buoy should fill
    info('circle_buoy.py main program')
    if auv is None:
        auv = control.AUV(cauv_node)

    b = BouyCircleObserver(cauv_node, auv, opts.strafe_speed, opts.buoy_size)
    b.run()

    info('circle_buoy.py complete')



def runStandalone(node_name = 'py-crcb', opts=None):
    node = cauv.node.Node('py-crcb')
    runWithNode(node, None, opts)

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option("-n", "--name", dest="name", default="py-crcb", help="CAUV Node name")
    p.add_option('-s', "--strafe-speed", dest="strafe_speed", default=20, type=int)
    p.add_option('-b', "--buoy-size", dest="buoy_size", default=20, type=int)
    opts, args = p.parse_args()

    cauv_node = cauv.node.Node(opts.name)
    runStandalone(opts.name, opts)

    

