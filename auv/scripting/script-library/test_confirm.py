from AI_classes import aiScript

class test_script(aiScript):
    def run(self):
        while True:
            if raw_input('Something was detected, has it been confirmed?'):
                self.exit(0)
                break
            

if __name__=='__main__':
    script = test_script('test')
    script.run()
