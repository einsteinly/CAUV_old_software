from AI_classes import aiScript, aiScriptOptions

from cauv.debug import info, warning, error, debug

import time

class scriptOptions(aiScriptOptions):
    initial = 'hello'
    variable = 0
    class Meta:
        dynamic = ['variable',]

class script(aiScript):
    def run(self):
        #self.request_pl('circle_buoy.pipe')
        self.auv.downlights(0)
        
        while True:
            i = raw_input('Continue? y/n/m: ')
            print self.auv.position
            if i.lower() == 'n':
                break
            elif i.lower() == 'm':
                self.request_control_and_wait()
                debug('calling self.auv.strafe()...')
                self.auv.strafe(42)
                debug('calling self.auv.prop()...')
                self.auv.prop(39)
                time.sleep(1)
                debug('resetting values to 0')
                self.auv.strafe(0)
                self.auv.prop(0)
                debug('self.auv: %s' % str(self.auv))
                self.drop_control()
            else:
                print self.options.initial, self.options.variable
                time.sleep(1)
        
        print 'Dropping pipe'
        #self.drop_pl('circle_buoy.pipe')
        a = raw_input('Something was detected, has it been confirmed? y/n: ')
        if a == 'y':
            return 'SUCCESS'
        elif a == 'n':
            return 'FAILURE'
        else:
            raise Exception
            

if __name__=='__main__':
    script = script('test')
    script.run()
