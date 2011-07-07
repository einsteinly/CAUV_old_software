from cauv.debug import debug, warning, error, info

import cauv.messaging as msg

import time
import optparse
import traceback

from AI_classes import aiProcess

class aiLocationProvider (msg.MessageObserver):
    def timeout():
        pass

    def fixPosition():
        pass

    def isFinished():
        pass
        
    def getPosition():
        pass


class aiLocation(aiProcess):
    def __init__(self, opts, args):
        aiProcess.__init__(self, 'location')
        self.options = opts
        self.timeout = False
    
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
            self.locationProvider = self.positioning.locationProvider(args)
            if not isinstance(self.locationProvider, aiLocationProvider):
                error("Not a subclass of aiLocationProvider")
                raise Exception
            self.addObserver(self.locationProvider)
        except Exception:
            error("%s doesn't contain a class called locationProvider" % scriptName)
        
    def run(self):
    
        # stop the auv from being moved by other threads
        self.ai.auv_control.pause(self.options.timeout)
        self.locationProviderThread = thread.start_new_thread(self.locationProvider.fixPosition, ())
            
        while True:
            if self.timedout:
                error("Position provider is taking too long to give a result. Killing it")
                self.locationProvider.timeout()
                pass; # TODO Kill the sonarPosition process
            while not self.locationProvider.isFinished():
                time.sleep(1) # check if its finished once per second
           
           # we now should have a fix ready for us
           (x, y) = self.locationProvider.getPosition()
           auv.send(msg.LocationMessage(x, y))
           time.sleep(self.options.wait)
                

    @external_function
    def pauseTimeout():
        self.timedout = True
        
    
if __name__ == '__main__':
    p = optparse.OptionParser()
    
    p.add_option('-w', '--wait', dest='wait', type="int", default=10, help="time to wait inbetween captures")
    p.add_option('-t', '--timeout', dest='timeout', type="int", default=20, help='maximum time to wait for a position fix')
    p.add_option('-s', '--script', dest='script', default="bay_processor", help='script to process sonar data')
    
    (opts, args) = p.parse_args()
    
    sc = sonarLocation(opts, args)
    sc.run()
