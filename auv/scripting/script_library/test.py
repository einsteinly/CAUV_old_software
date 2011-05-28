from AI_classes import aiScript

class script(aiScript):
    def run(self):
        while True:
            if raw_input('Something was detected, has it been confirmed?'):
                self.notify_exit(0)
                break
            

if __name__=='__main__':
    script = script('test')
    script.run()
