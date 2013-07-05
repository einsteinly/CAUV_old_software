#!/usr/bin/env python2.7
from cauv import messaging
from cauv.debug import debug, error, warning, info
from collections import deque

import time

import AI
from utils import event

class SimpleBuoyDetector(event.EventLoop, AI.Detector):
    class DefaultOptions(AI.Detector.DefaultOptions):
        def __init__(self):
            self.Sightings_Period = 1.0
            self.Required_Sightings = 5
            self.Max_Horizontal_Variation = 0.7
            self.Max_Vertical_Variation = 0.4
            self.Max_Samples = 7
            self.Max_Sample_Time = 1.0
            self.Pipeline_Name = "simple_detect_buoy"
            self.Circles_Name = "simple_buoy"
            
    class Debug(AI.Detector.Debug):
        def __init__(self):
            self.value = False
            
    def __init__(self):
        event.EventLoop.__init__(self)
        AI.Detector.__init__(self)
        self.load_pipeline(self.options.Pipeline_Name)
        self.node.subMessage(messaging.CirclesMessage())
        self.node.addObserver(self)
        self.log('Looking for the buoy')
        self.circles = deque(maxlen = self.options.Max_Samples)
        self.times = deque(maxlen = self.options.Max_Samples)
    
    @event.event_func
    def onCirclesMessage(self, m):
        if m.name != self.options.Circles_Name:
            return
        #check right number of circles
        if len(m.circles) != 1:
            debug("Incorrect number of circles", 5)
            return
        debug("Saw circle", 5)
        self.circles.append(m.circles[0])
        self.times.append(time.time())
    
    @event.repeat_event(0.2, True)
    def process(self):
        now_t = time.time()
        no_samples = len(self.times)
        #Find first sample in time frame
        #To get enough samples, know must be atleast 1 in the first no_samples-max_samples+1 of samples
        for sample_i in range(no_samples-self.options.Required_Sightings+1):
            sample_t = now_t-self.times[sample_i]
            if sample_t < self.options.Max_Sample_Time:
                start_i = sample_i
                break
        else:
            debug("Not enough circles in time.", 5)
            self.value = False
            self.fire(0)
            return False
        #Order samples by horizontal position
        sorted_samples = sorted(list(self.circles)[start_i:], key=lambda circ: circ.centre.x)
        #for each selection in the correct horizontal variation, repeat for vertical position
        for substart_i in range(len(sorted_samples)-self.options.Required_Sightings):
            for subend_i in range(len(sorted_samples)-1, substart_i+self.options.Required_Sightings-1, -1):
                debug("Horizontal {}".format(sorted_samples[subend_i].centre.x-sorted_samples[substart_i].centre.x), 5)
                if sorted_samples[subend_i].centre.x-sorted_samples[substart_i].centre.x>self.options.Max_Horizontal_Variation:
                    #samples too far apart, move back 1
                    continue
                #found largest subset starting at substart_i
                sorted_samples_vert = sorted(sorted_samples[substart_i:subend_i], key=lambda circ: circ.centre.y)
                for substart_vert_i in range(len(sorted_samples_vert)-self.options.Required_Sightings):
                    debug("Vertical {}".format(sorted_samples_vert[substart_vert_i+self.options.Required_Sightings].centre.y-sorted_samples_vert[substart_vert_i].centre.y), 5)
                    if sorted_samples_vert[substart_vert_i+self.options.Required_Sightings].centre.y-sorted_samples_vert[substart_vert_i].centre.y<self.options.Max_Vertical_Variation:
                        self.value = True
                        self.fire(1)
                        return True
                break
        self.value = False
        self.fire(0)
        return False
        
        
Detector = SimpleBuoyDetector

if __name__ == "__main__":
    SimpleBuoyDetector.entry()