import traceback
import optparse
import time
import subprocess
import threading
import os
import sys
import string
import psi.process
#import psutil

from cauv.debug import debug, info, warning, error

from utils.multitasking import spawnDaemon

CPU_Poll_Time = 0.025
Poll_Delay = 2.0
Exe_Prefix = '' # set these using command line options
Script_Dir = '' #
Mem_Divisor = 1024*1024 # MB
Mem_Units = 'M'         # MB

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
        self.status = ''
        self.priority = 0
        self.cpu = None
        self.mem = None
        self.threads = None
        self.error = None
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
        CAUVTask('img-pipe default',     'nohup %EDIR/img-pipeline -n default',         True, ['img-pipeline -n default']),
        CAUVTask('img-pipe buoy-detect', 'nohup %EDIR/img-pipeline -n buoy-detect',     True, ['img-pipeline -n buoy-detect']),
        CAUVTask('img-pipe pipe-detect', 'nohup %EDIR/img-pipeline -n pipe-detect',     True, ['img-pipeline -n pipe-detect']),
        CAUVTask('img-pipe sonar',       'nohup %EDIR/img-pipeline -n sonar',           True, ['img-pipeline -n sonar']),
        CAUVTask('sonar',                'nohup %EDIR/sonar /dev/ttyUSB1',              True, ['sonar']),
        CAUVTask('controlv2',            'nohup %EDIR/controlv2 -m/dev/ttyUSB0 -x0',    True, ['controlv2']),
        CAUVTask('spread',               'nohup spread',                                True, ['spread']),
        CAUVTask('persist',              'nohup /bin/sh %SDIR/run.sh %SDIR/persist.py',         False, ['persist.py']),
        CAUVTask('battery',              'nohup /bin/sh %SDIR/run.sh %SDIR/battery_monitor.py', False, ['battery_monitor.py']),
        CAUVTask('sonar log',       '', False, ['sonarLogger.py']),
        CAUVTask('telemetry log',   '', False, ['telemetryLogger.py']),
        CAUVTask('watch',           '', False, ['watch.py']),
        CAUVTask('AI Manager',      '', False, ['AI_manager']),
        CAUVTask('AI Ctrl Manager', '', False, ['AI_control_manager']),
        CAUVTask('AI Detectors',    '', False, ['AI_detection_process']),
        CAUVTask('AI Task Manager', '', False, ['AI_task_manager'])
]

def limitLength(string, length=48):
    string = string.replace('\n', '\\ ')
    if length >= 3 and len(string) > length:
        string = string[:length-3] + '...'
    return string

def psiProcStatusToS(n):
    'return string describing process status value'
    if n == psi.process.PROC_STATUS_SIDL:
        return 'idle'
    elif n == psi.process.PROC_STATUS_SRUN:
        return 'running'
    elif psi.process.PROC_STATUS_SSLEEP:
        return 'sleeping'
    elif psi.process.PROC_STATUS_SSTOP:
        return 'stopped'
    elif psi.process.PROC_STATUS_SZOMB:
        return 'zombie'
    else:
        return '?'

def getProcesses():
    # returns dictionary of short name : CAUVTasks, all fields filled in
    all_processes = psi.process.ProcessTable()
    # find the pids we're interested in
    processes = {}
    short_names = {}
    for p in processes_to_start:
        processes[p.shortName()] = p
        short_names[p.command()] = p.shortName()
    for pid, process in all_processes.iteritems():
        try:
            tried_to_get_info = False
            task = None
            if process.command in short_names:
                debug('found a process of interest: %s is %s' %
                    (short_names[process.command], process.command))
                task = processes[short_names[process.command]]
            else:
                try:
                    for p_to_start in processes.values():
                        for name in p_to_start.searchForNames():
                            if process.command.find(name) != -1:
                                task = processes[p_to_start.shortName()]
                                raise Exception('break')
                except Exception, e:
                    if str(e) != 'break':
                        raise
            if task is not None:
                process.refresh()
                task.error = None
                task.process = process
                task.running_command = process.command
                task.status = psiProcStatusToS(process.status)
                # these might cause privileges exception
                tried_to_get_info = True
                task.priority = process.priority
                task.cpu = process.pcpu
                task.mem = process.rss # resident size: process.vsz is the virtual mem size
                task.threads = process.nthreads
        except psi.AttrInsufficientPrivsError:
            if tried_to_get_info:
                task.error = 'Insufficient Privileges'
        except Exception, e:
            warning('%s\n%s' % (e, traceback.format_exc()))
    return processes

def printDetails(cauv_task_list, more_details=False):
    Format_Short = '%23s %7s'
    Format_Extra = '%7s %7s %7s %7s %s'
    header = Format_Short % ('name', 'status')
    if more_details:
        header += Format_Extra % ('pid', 'CPU', 'Mem', 'Threads', 'Command')
    info(header)
    info('-' * len(header.expandtabs()))
    for cp in cauv_task_list.values():
        line = Format_Short % (cp.shortName() , cp.status)
        if more_details and cp.process is not None:
            cpus = 'None'
            mems = 'None'
            if cp.cpu is not None: cpus = '%4.2f' % cp.cpu
            if cp.mem is not None: mems = '%4.1f%s' % (cp.mem / Mem_Divisor, Mem_Units)
            line += Format_Extra % (
                cp.process.pid, cpus, mems, cp.threads,
                limitLength(cp.running_command)
            )
            if cp.error is not None:
                line += '\t(Error: %s)' % cp.error
                
        info(line)
    info(' ' * len(header.expandtabs()))

def broadcastStatus(cauv_node):
    debug('TODO: send this as a message')
    info('load: %s (1min) %s (5min) %s (15min)' % psi.loadavg())
    info('uptime: %s hours' % (psi.uptime().tv_sec / 3600))

def broadcastDetails(processes, cauv_node):
    import cauv.messaging as msg
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
        import cauv.node as node
        cauv_node = node.Node("watch")

    try:
        while True:
            processes = getProcesses()
            printDetails(processes, opts.details)
            if cauv_node is not None:
                broadcastDetails(processes, cauv_node)
                broadcastStatus(cauv_node)
            if not opts.no_start:
                startInactive(processes)
            time.sleep(Poll_Delay)
            if not opts.persistent:
                break
    except KeyboardInterrupt:
        info('Interrupt caught, exiting...')

