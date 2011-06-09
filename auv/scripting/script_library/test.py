from AI_classes import aiScript

class script(aiScript):
    def run(self):
        while True:
            for opt in self.options.__dict__:
                print opt,self.options.__dict__[opt]
            if raw_input('Something was detected, has it been confirmed?'):
                self.notify_exit(0)
                break
            

if __name__=='__main__':
    script = script('test')
    script.run()
