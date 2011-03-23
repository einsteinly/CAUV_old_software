import traceback
import psutil
import optparse
import time
import subprocess
import os
import cauv.messaging as msg
import cauv.node as node

from cauv.debug import debug, info, warning, error

pid_list = None

class CAUVTask:
    def __init__(self, name, command, restart=True, names=[]):
        self.__short_name = name
        self.__command = command
        self.__restart = restart
        self.__names = names
        self.process = None
        self.running_command = None
        self.status = 'unknown'
        self.cpu = None
        self.mem = None
        self.threads = None
    def command(self):
        return self.__command
    def shortName(self):
        return self.__short_name
    def searchForNames(self):
        return self.__names
    def doStart(self):
        return self.__restart
    def start(self):
        if(self.__restart):
            # stdout is logged, so dump it to /dev/null
            subprocess.Popen(self.__command.split(' '),
                             #stdout=subprocess.DEVNULL, only in python 3.3
                             stdout=open('/dev/null', 'w'),
                             stderr=open('%s-stderr.log' % self.shortName(), 'a'))

# global variable, set using command line options
cmd_prefix = '/usr/local/bin/'

# ---------------------------------------------------------------
# ---------- List of processes to start / monitor ---------------
# ---------------------------------------------------------------
processes_to_start = [
        CAUVTask(
            'remote',     # short name
            'nohup /bin/sh ./run.sh ./remote.py', # command
            True,         # do start/restart this process
            ['remote.py'] # list of names to search for in processes
        ),
        CAUVTask('logger',   'nohup /bin/sh ./run.sh ./logger.py',        True,  ['logger.py']),
        CAUVTask('img-pipe', 'nohup %sauv/bin/img-pipeline' % cmd_prefix, True,  ['img-pipeline']),
        CAUVTask('sonar',    'nohup %sauv/bin/sonar' % cmd_prefix,        True,  ['sonar']),
        CAUVTask('control',  'nohup %sauv/bin/control' % cmd_prefix,      False, ['control']),
        CAUVTask('spread',   'nohup spread',                              True,  ['spread']),
        CAUVTask('watch',    '',                                          False, ['watch.py'])
        CAUVTask('watch',    'nohup /bin/sh ./run.sh ./battery_monitor.py', True,  ['battery_monitor.py']) ]

def getProcesses():
    # returns dictionary of short name : CAUVTasks, all fields filled in
    pids = psutil.get_pid_list()
    # find the pids we're interested in
    processes = {}
    short_names = {}
    for p in processes_to_start:
        processes[p.shortName()] = p
        short_names[p.command()] = p.shortName()
    for pid in pids:
        try:
            task = None
            p = psutil.Process(pid)
            command_str = ' '.join(p.cmdline)
            if command_str in short_names:
                task = processes[short_names[command_str]]
            else:
                try:
                    for p_to_start in processes.values():
                        for name in p_to_start.searchForNames():
                            if command_str.find(name) != -1:
                                task = processes[p_to_start.shortName()]
                                raise Exception('break')
                except Exception, e:
                    #print e
                    if str(e) != 'break':
                        raise
            if task is not None:
                #debug(task.shortName())
                task.process = p
                task.running_command = command_str
                task.status = p.status
                task.cpu = p.get_cpu_percent()
                task.mem = p.get_memory_percent()
                task.threads = p.get_num_threads()
        except psutil.AccessDenied:
            pass
        except Exception, e:
            warning('%s\n%s' % (e, traceback.format_exc()))
    return processes

def printDetails(cauv_task_list, more_details=False):
    header = 'name    \tstatus  '
    if more_details:
        header += '\tpid     \tCPU%\tMem%\tthreads \tcommand '
    info(header)
    info('-' * len(header.expandtabs()))
    for cp in cauv_task_list.values():
        line = '%8s\t%8s' % (cp.shortName() , cp.status)
        if more_details and cp.process is not None:
            if cp.threads is None:
                line += '\t(access denied)'
            else:
                line += '\t%8s\t%4.2f\t%4.2f\t%8s\t%8s' % (
                    cp.process.pid,
                    cp.cpu,
                    cp.mem,
                    cp.threads,
                    cp.running_command
                )
        info(line)
    info('-' * len(header.expandtabs()))

def broadcastDetails(processes, cauv_node):
    for cp in processes.values():
        m = msg.ProcessStatusMessage()
        m.process = cp.shortName()
        m.status = str(cp.status)
        if cp.cpu is None:
            m.cpu = -1
        else:
            m.cpu = float(cp.cpu)
        if cp.mem is None:
            m.mem = -1
        else:
            m.mem = float(cp.mem)
        if cp.threads is None:
            m.threads = 0
        else:
            m.threads = cp.threads
        cauv_node.send(m)

def startInactive(processes):
    updateProcessesByCommand()
    for pc in processes:
        if processes[pc] is None:
            cauv_task = processes_by_command[pc]            
            if cauv_task.doStart():
                info('starting %s (%s)' % (shortName(pc), pc))
                cauv_task.start()

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-N', '--no-start', dest='no_start', default=False,
                 action='store_true', help="Don't start processes, just" +\
                 " report whether processes that would be started are running")
    p.add_option('-d', '--show-details', dest='details', default=True,
                 action='store_true', help='display additional details')
    p.add_option('-n', '--no-show-details', dest='details',
                 action='store_false', help='hide additional details')
    p.add_option('-p', '--persistent', dest='persistent', default=False,
                 action='store_true', help='keep going until stopped')
    p.add_option('--no-broadcast', dest='broadcast', default=True,
                 action='store_false', help="don't broadcast messages "+\
                 'containing information on running processes')
    p.add_option('-e', '--exec-prefix', dest='exec_prefix',
                 default='/usr/local/bin/', type=str,
                 action='store', help='exec prefix for cauv executables ' +\
                 '(e.g., /usr/local/bin')

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)

    global exec_prefix
    exec_prefix = opts.exec_prefix

    if opts.broadcast:
        cauv_node = node.Node("watch")
    
    if opts.no_start:
        while True:
            processes = getProcesses()
            printDetails(processes, opts.details)
            if cauv_node:
                broadcastDetails(processes, cauv_node)
            time.sleep(3.0)
            if not opts.persistent:
                break
    else:
        while True:
            processes = getProcesses()
            printDetails(processes, opts.details)
            if cauv_node:
                broadcastDetails(processes, cauv_node)
            startInactive(processes)
            time.sleep(3.0)
            if not opts.persistent:
                break

