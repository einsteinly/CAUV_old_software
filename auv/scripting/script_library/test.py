from AI_classes import aiScript, aiScriptOptions

import time

class scriptOptions(aiScriptOptions):
    initial = 'hello'
    variable = 0
    class Meta:
        dynamic = ['variable',]

class script(aiScript):
    def run(self):
        while raw_input('Continue? y/n: ') != 'n':
            print self.options.initial, self.options.variable
            time.sleep(1)
        a = raw_input('Something was detected, has it been confirmed? y/n: ')
        if a == 'y':
            self.notify_exit('SUCCESS')
        elif a == 'n':
            self.notify_exit('FAILURE')
        else:
            raise Exception
            

if __name__=='__main__':
    script = script('test')
    script.run()
