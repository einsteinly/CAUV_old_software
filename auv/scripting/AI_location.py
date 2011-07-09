from cauv.debug import debug, warning, error, info
from cauv import control

import cauv.messaging as msg

import time
import math
import optparse
import traceback
import threading


from AI_classes import aiProcess, external_function


class vec:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return vec(self.x+other.x, self.y+other.y)
    def __sub__(self, other):
        return vec(self.x-other.x, self.y-other.y)
    def __abs__(self):
        return math.sqrt(dotProd(self,self))
    def __repr__(self):
        return "(%f,%f)"%(self.x,self.y)
    
def dotProd(p1,p2):
    return p1.x*p2.x + p1.y*p2.y

class aiLocationProvider(msg.MessageObserver):
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
        self.node = node
        self.node.addObserver(self)
        # events
        self.finished = threading.Event()
        self.performFix = threading.Event()
        self.stopped = threading.Event()
        
    def fixPosition(self):
        # do your work in here
        # if self.stop.is_set() is ever true you sould
        # stop processing and return
        pass
    
    def getPosition(self):
        # return your results from here as (x, y)
        pass
        
    def stop(self):
        self.stopped.set()
        pass

    def startFix(self):
        self.performFix.set()

    def run(self):
        while True:
            # wait until we're told to run
            self.performFix.clear()
            self.performFix.wait()
            
            # run the fix
            self.stopped.clear()
            self.finished.clear()
            self.fixPosition()
            self.finished.set()

    def isFinished(self):
        # if stop is set then processing might not have actually finished
        if(self.stopped.is_set()):
            return False
        return self.finished.is_set()
        

class aiLocation(aiProcess):
    def __init__(self, opts, args):
        aiProcess.__init__(self, 'location')
        self.options = opts
        self.args = args
        self.timeout = threading.Event()
        self.auv = control.AUV(self.node)
        self._register()
        
    def run(self):
        self.ai.auv_control.pause(self.process_name, self.options.timeout)
    
        scriptName = self.options.script
    
        # import the file passed on the command line 
        try:
            self.positioning = __import__('positioning.'+ scriptName, fromlist=['positioning'])
        except Exception:
            error("Could not import positioning script " + scriptName)
            traceback.print_exc()
            raise Exception
        
        # create an instance
        try:
            self.locator = self.positioning.locationProvider(self.node, self.args)
        except Exception:
            traceback.print_exc()
            error("%s doesn't contain a class called locationProvider" % scriptName)
            return
        
        self.locationProviderThread = threading.Thread(target=self.locator.run)
        self.locationProviderThread.setDaemon(True)
        self.locationProviderThread.start()
            
    
        while True:
            self.timeout.clear()
        
            # stop the auv from being moved by other threads
            self.ai.auv_control.pause(self.process_name, self.options.timeout)
                        
            info("AI_location: Running fix")
            self.locator.startFix()
            
            
            while not self.locator.isFinished():
                if self.timeout.is_set():
                    error("AI_location: Position provider is taking too long to give a result. Killing it")
                    self.locator.stop()
                    break;
                else:
                    time.sleep(1) # check if its finished once per second
           
            if self.locator.isFinished():
                # we now should have a fix ready for us
                position = self.locator.getPosition()
                if position is None:
                    info("AI_location: Positioner returned None")
                else: info("AI_location: Positioner returned: %f, %f" % (position.x, position.y))
                self.auv.send(msg.SonarLocationMessage(msg.floatXY(position.x, position.y))) #pylint: disable=E1101
            
            info("AI_location: Waiting for %i seconds before next fix attemp" % self.options.wait)
            time.sleep(self.options.wait)                

    @external_function
    def onPauseTimeout(self):
        error("AI_location: Pause time has run out")
        self.timeout.set()
        
    
if __name__ == '__main__':
    p = optparse.OptionParser()
    
    p.add_option('-w', '--wait', dest='wait', type="int", default=30, help="time to wait inbetween captures")
    p.add_option('-t', '--timeout', dest='timeout', type="int", default=15, help='maximum time to wait for a position fix')
    p.add_option('-s', '--script', dest='script', default="bay_processor", help='script to process sonar data')
    
    (opts, args) = p.parse_args()
    
    sc = aiLocation(opts, args)
    sc.run()
