from AI_classes import aiScript, aiScriptOptions

import time

class scriptOptions(aiScriptOptions):
    initial = 'hello'
    variable = 0
    class Meta:
        dynamic = ['variable',]

class script(aiScript):
    def run(self):
        try:
            while True:
                print self.options.initial, self.options.variable
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        if raw_input('Something was detected, has it been confirmed?'):
            self.notify_exit(0)
            

if __name__=='__main__':
    script = script('test')
    script.run()
