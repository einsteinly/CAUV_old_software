import traceback
import psutil
import optparse
import time
import subprocess
import threading
import os
import sys
import string

import cauv.messaging as msg
import cauv.node as node
from cauv.debug import debug, info, warning, error

CPU_Poll_Time = 0.025
Poll_Delay = 2.0
Exe_Prefix = '' # set these using command line options
Script_Dir = '' #


def spawnDaemon(func):
    # see http://code.activestate.com/recipes/66012-fork-a-daemon-process-on-unix/
    # do the UNIX double-fork magic, see Stevens' "Advanced
    # Programming in the UNIX Environment" for details (ISBN 0201563177)
    try:
        pid = os.fork()
        if pid > 0:
            # parent process
            return
    except OSError, e:
        print >>sys.stderr, "fork #1 failed: %d (%s)" % (e.errno, e.strerror)
        sys.exit(1)

    # decouple from parent environment
    # ... don't
    #os.chdir("/")
    os.setsid()
    #os.umask(0)

    # do second fork
    try:
        pid = os.fork()
        if pid > 0:
            # exit from second parent
            sys.exit(0)
    except OSError, e:
        print >>sys.stderr, "fork #2 failed: %d (%s)" % (e.errno, e.strerror)
        sys.exit(1)
    # do stuff
    func()

    # all done
    os._exit(os.EX_OK)

def fillPathPlaceholders(s):
    r = string.replace(s, '%EDIR', Exe_Prefix)
    return string.replace(r, '%SDIR', Script_Dir)

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
            spawnDaemon(self.__start)
    def __start(self):
            # stdout is logged, so dump it to /dev/null
            subprocess.Popen(fillPathPlaceholders(self.__command).split(' '),
                             #stdout=subprocess.DEVNULL, only in python 3.3
                             stdout=open('/dev/null', 'w'),
                             stderr=open('%s-stderr.log' % self.shortName(), 'a'))

# ---------------------------------------------------------------
# ---------- List of processes to start / monitor ---------------
# ---------------------------------------------------------------
processes_to_start = [
        CAUVTask(
            'remote',     # short name
            'nohup /bin/sh %SDIR/run.sh %SDIR/remote.py', # command: %SDIR expands to --script-dir, %EDIR to --exec-prefix
            True,         # do start/restart this process
            ['remote.py'] # list of names to search for in processes
        ),
        CAUVTask('logger',               'nohup /bin/sh %SDIR/run.sh %SDIR/logger.py',  True, ['logger.py']),
        CAUVTask('img-pipe default',     'nohup %EDIR/img-pipeline',                    True, ['img-pipeline -n default']),
        CAUVTask('img-pipe buoy-detect', 'nohup %EDIR/img-pipeline -n buoy-detect',     True, ['img-pipeline -n buoy-detect']),
        CAUVTask('img-pipe pipe-detect', 'nohup %EDIR/img-pipeline -n pipe-detect',     True, ['img-pipeline -n pipe-detect']),
        CAUVTask('img-pipe sonar',       'nohup %EDIR/img-pipeline -n sonar',           True, ['img-pipeline -n sonar']),
        CAUVTask('sonar',                'nohup %EDIR/sonar /dev/ttyUSB1',              True, ['sonar']),
        CAUVTask('controlv2',            'nohup %EDIR/controlv2 -m/dev/ttyUSB0 -x0',    True, ['controlv2']),
        CAUVTask('spread',               'nohup spread',                                True, ['spread']),
        CAUVTask('persist',              'nohup /bin/sh %SDIR/run.sh %SDIR/persist.py',         False, ['persist.py']),
        CAUVTask('battery',              'nohup /bin/sh %SDIR/run.sh %SDIR/battery_monitor.py', False, ['battery_monitor.py']),
        CAUVTask('watch',           '', False, ['watch.py']),
        CAUVTask('AI Manager',      '', False, ['AI_manager']),
        CAUVTask('AI Ctrl Manager', '', False, ['AI_control_manager']),
        CAUVTask('AI Detectors',    '', False, ['AI_detection_process']),
        CAUVTask('AI Task Manager', '', False, ['AI_task_manager'])
]

def limitLength(string, length=40):
    string = string.replace('\n', '\\ ')
    if length >= 3 and len(string) > length:
        string = string[:length-3] + '...'
    return string

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
            process = psutil.Process(pid)
            command_str = ' '.join(process.cmdline)
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
                task.process = process
                task.running_command = command_str
                task.status = process.status
                task.cpu = process.get_cpu_percent(CPU_Poll_Time)
                task.mem = process.get_memory_percent()
                task.threads = process.get_num_threads()
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
                    limitLength(cp.running_command)
                )
        info(line)
    info(' ' * len(header.expandtabs()))

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

def startInactive(cauv_task_list):
    for cauv_task in processes.values():
        if cauv_task.process is None:
            if cauv_task.doStart():
                info('starting %s (%s)' % (cauv_task.shortName(),
                     fillPathPlaceholders(cauv_task.command())))
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
    p.add_option('-q', '--no-broadcast', dest='broadcast', default=True,
                 action='store_false', help="don't broadcast messages "+\
                 'containing information on running processes')
    p.add_option('-e', '--exec-prefix', dest='cmd_prefix',
                 default='/usr/local/bin/cauv/', type=str,
                 action='store', help='exec prefix for cauv executables ' +\
                 '(e.g., /usr/local/bin')
    p.add_option('-s', '--script-dir', dest='script_dir', default='./',
                 type=str, action='store',
                 help='script directory (where to find run.sh)')

    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)

    Exe_Prefix = opts.cmd_prefix
    Script_Dir = opts.script_dir

    cauv_node = None
    if opts.broadcast:
        cauv_node = node.Node("watch")

    try:
        while True:
            processes = getProcesses()
            printDetails(processes, opts.details)
            if cauv_node is not None:
                broadcastDetails(processes, cauv_node)
            if not opts.no_start:
                startInactive(processes)
            time.sleep(Poll_Delay)
            if not opts.persistent:
                break
    except KeyboardInterrupt:
        info('Interrupt caught, exiting...')

