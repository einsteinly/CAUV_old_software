import subprocess
import time
"""
Processes
detector process - runs detector classes, and loops through processing
task manager - listens to detector messages, and manages running scripts
emergency control - runs background checking and can override script in auv_control
scripts - do stuff
auv control - listens to messages from scripts and emergency control

main process keeps sets up things alive
"""
#TODO cope with restarts
class Process():
    def __init__(self, command):
        self.command = command
        self.start()
    def start(self):
        print self.command
        self.process = subprocess.Popen(self.command)
    def status(self):
        if self.process.poll():
            return 0
        return 1
    def terminate(self):
        pass

process_data_list = (
            ('auv_control', '/bin/sh ./run.sh ./AI_control_manager.py'),
            ('detector_control', '/bin/sh ./run.sh ./AI_detection_process.py'),
            #('task_control', '/bin/sh ./run.sh ./AI_task_manager.py'),
            )

class AImanager:
    def __init__(self):
        self.processes = {}
        for process_data in process_data_list:
            self.processes[process_data[0]] = Process(process_data[1].split(' '))
    def run(self):
        while True:
            for process_name in self.processes:
                process = self.processes[process_name]
                if process.status() == 0:
                    process.start()
            time.sleep(2)
if __name__ == '__main__':
    ai = AImanager()
    ai.run()
