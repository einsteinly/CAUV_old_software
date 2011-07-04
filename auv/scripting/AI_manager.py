import subprocess
import time
import optparse

from cauv.debug import info
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
    def __init__(self, command, opts):
        self.command = command
        self.opts = opts
        self.start()
    def start(self):
        c = ' '.join(self.command)+' '+' '.join(['--%s=%s' %(x[0],str(x[1])) if not isinstance(x[1], bool) else ('--'+x[0] if x[1] else '') for x in self.opts.items()])
        info('Running command: '+c)
        if 'restore' in self.opts: self.opts['restore'] = True #if the process stops we want to try and restore it with its old data
        self.process = subprocess.Popen(c.split(' '))
    def status(self):
        if self.process.poll():
            return 0
        return 1
    def terminate(self):
        pass

process_data_list = (
            ('auv_control', '/bin/sh ./run.sh ./AI_control_manager.py', []),
            ('detector_control', '/bin/sh ./run.sh ./AI_detection_process.py', []),
            ('task_control', '/bin/sh ./run.sh ./AI_task_manager.py', ['mission', 'restore']),
            ('pipeline_control', '/bin/sh ./run.sh ./AI_pipeline_manager.py', ['disable_gui']),
            )

class AImanager():
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.processes = {}
        for process_data in process_data_list:
            self.processes[process_data[0]] = Process(process_data[1].split(' '),opts=dict([(x,self.kwargs[x]) for x in process_data[2]]))
    def run(self):
        while True:
            for process_name in self.processes:
                process = self.processes[process_name]
                if process.status() == 0:
                    process.start()
            time.sleep(2)
if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_option('-m', '--mission', dest='mission', default='mission',
                 type=str, action='store', help='which mission script to run (default = mission)')
    p.add_option('-g', '--disable-gui', dest='disable_gui', default=False,
                 action='store_true', help="disable/ignore gui output nodes")
    opts, args = p.parse_args()
    #unfortunately opts looks like dict but is not. fortunately opts.__dict__ is.
    ai = AImanager(**opts.__dict__)
    ai.run()
