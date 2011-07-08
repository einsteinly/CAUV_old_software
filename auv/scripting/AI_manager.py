import subprocess
import time
import optparse
import cPickle
import threading

from cauv.debug import info, debug, warning, error
from cauv import messaging
from cauv.node import Node

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
        self.started = threading.Event()
    def start(self):
        c = ' '.join(self.command)+' '+' '.join(['--%s=%s' %(x[0],str(x[1])) if not isinstance(x[1], bool) else ('--'+x[0] if x[1] else '') for x in self.opts.items()])
        info('Running command: '+c)
        if 'restore' in self.opts: self.opts['restore'] = True #if the process stops we want to try and restore it with its old data
        self.process = subprocess.Popen(c.split(' '))
        self.started.wait(5)
        if not self.started.is_set():
            warning('Process %s did not respond in time after it was started, may be dead or still starting.' %(self.command,))
    def status(self):
        if self.process.poll():
            return 0
        return 1
    def terminate(self):
        pass

process_data_list = (
            ('pipeline_manager', '/bin/sh ./run.sh ./AI_pipeline_manager.py', ['disable_gui', 'reset_pls', 'freeze_pls', 'restore']),
            ('auv_control', '/bin/sh ./run.sh ./AI_control_manager.py', []),
            ('task_manager', '/bin/sh ./run.sh ./AI_task_manager.py', ['mission', 'restore']),
            ('detector_control', '/bin/sh ./run.sh ./AI_detection_process.py', ['disable_control']),
            )

class AImanager(messaging.MessageObserver):
    def __init__(self, **kwargs):
        messaging.MessageObserver.__init__(self)
        self.node = Node("aimanage")
        self.node.join("ai")
        self.kwargs = kwargs
        if 'disable' in self.kwargs:
            disable = self.kwargs.pop('disable')
        self.processes = {}
        for process_data in process_data_list:
            if not process_data[0] in disable:
                self.processes[process_data[0]] = Process(process_data[1].split(' '),opts=dict([(x,self.kwargs[x]) for x in process_data[2]]))
        self.node.addObserver(self)
    def run(self):
        for process_name in self.processes:
            self.processes[process_name].start()
        while True:
            for process_name in self.processes:
                process = self.processes[process_name]
                if process.status() == 0:
                    process.started.clear()
                    process.start()
                time.sleep(0.5)
            time.sleep(2)
    def onAIMessage(self, m):
        message = cPickle.loads(m.msg)
        if message[0] == 'STATE':
            if message[2] == 'REGISTER':
                try:
                    self.processes[message[1]].started.set()
                except KeyError:
                    pass
if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_option('-m', '--mission', dest='mission', default='mission',
                 type=str, action='store', help='which mission script to run (default = mission)')
    p.add_option('--disable-gui', dest='disable_gui', default=False,
                 action='store_true', help="disable/ignore gui output nodes")
    p.add_option('--disable-control', dest='disable_control', default=False,
                 action='store_true', help="stop AI script from controlling the sub")
    p.add_option('--disable', dest='disable', default=[],
                 type=str, action='append', help="disable process by name")
    p.add_option('--reset-pls', dest='reset_pls', default=False,
                 action='store_true', help="reset pipelines to those stored in /pipelines")
    p.add_option('--freeze-pls', dest='freeze_pls', default=False,
                 action='store_true', help="ignore changes to the pipeline")
    opts, args = p.parse_args()
    #unfortunately opts looks like dict but is not. fortunately opts.__dict__ is.
    ai = AImanager(**opts.__dict__)
    ai.run()
