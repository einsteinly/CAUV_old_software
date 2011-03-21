import traceback
import psutil
import optparse
import time
import subprocess
import os

from cauv.debug import debug, info, warning, error


class CAUVTask:
    def __init__(self, name, command, restart=True, names=[]):
        self.__short_name = name
        self.__command = command
        self.__restart = restart
        self.__names = names
    def command(self):
        return self.__command
    def shortName(self):
        return self.__short_name
    def searchForNames(self):
        return self.__names
    def start(self):
        subprocess.Popen(self.__command.split(' '))

#TODO we have some sort of cauv install target?
if os.uname()[1] == 'red-herring':
    cmd_prefix = '/home/cauv/cmm/'
elif os.uname()[1].find('James'):
    cmd_prefix = '/Users/james/Development/cauv/cmm/'
else:
    cmd_prefix = ''

processes_to_start = [
        #       short-name, command,                       restart?, candidate      names
        CAUVTask('remote', 'nohup /bin/sh ./run.sh ./remote.py > remote-stdout.log 2> remote-stderr.log < /dev/null', True, ['remote.py']),
        CAUVTask('logger', 'nohup /bin/sh ./run.sh ./logger.py > logger-stdout.log 2> logger-stderr.log < /dev/null', True, ['logger.py']),
        CAUVTask('img-pipe', 'nohup %sauv/bin/img-pipeline  > img-pipe-stdout.log 2> img-pipe-stderr.log < /dev/null' % cmd_prefix, True, ['img-pipeline']),
        CAUVTask('sonar', 'nohup %sauv/bin/sonar > sonar-stdout.log 2> sonar-stderr.log < /dev/null' % cmd_prefix, True, ['sonar']),
        #CAUVTask('control', 'nohup %sauv/bin/control' % cmd_prefix, True, ['control']),
        CAUVTask('spread', 'nohup spread', True, ['spread'])
]

processes_by_command = {}

def updateProcessesByCommand():
    global processes_by_command
    processes_by_command = {}
    for p in processes_to_start:
        processes_by_command[p.command()] = p

def shortName(command):
    for p in processes_to_start:
        if p.command() == command:
            return p.shortName()
    return command

def getProcesses():
    # returns dict command_str : (Process object : actual command string)
    pids = psutil.get_pid_list()
    # find the pids we're interested in
    processes = {}
    for p in processes_to_start:
        processes[p.command()] = None
    for pid in pids:
        try:
            p = psutil.Process(pid)
            #parent = p.parent
            #debug('%s %s %s, parent: %s %s %s' % (
            #    p.exe, p.name, p.cmdline,
            #    parent.exe, parent.name, parent.cmdline))
            command_str = ' '.join(p.cmdline)
            if command_str in processes:
                processes[command_str] = (p, command_str)
            else:
                for p_to_start in processes_to_start:
                    for name in p_to_start.searchForNames():
                        if command_str.find(name) != -1:
                            processes[p_to_start.command()] = (p, command_str)
            #debug(command_str)
        except Exception, e:
            #warning('%s\n%s' % (e, traceback.format_exc()))
            pass
    return processes

def printDetails(processes, more_details=False):
    header = 'name\tstatus'
    if more_details:
        header += '\tstatus   \tpid\tCPU%\tMem%\tthreads\tcommand'
    info(header)
    info('-' * len(header.expandtabs()))
    for pc in processes:
        p, actual_command = processes[pc]
        if p is None:
            status = 'unknown'
        else:
            status = 'active'
        line = '%s\t%s' % (shortName(pc), status)
        if more_details and p is not None:
            line += '\t%s\t%s\t%.2f\t%.2f\t%s\t%s' % (
                p.status,
                p.pid,
                p.get_cpu_percent(),
                p.get_memory_percent(),
                p.get_num_threads(),
                actual_command
            )
        info(line)
 
def startInactive(processes):
    updateProcessesByCommand()
    for pc in processes:
        if processes[pc] is None:
            info('starting %s (%s)' % (shortName(pc), pc))
            cauv_task = processes_by_command[pc]
            cauv_task.start()

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-N', '--no-start', dest='no_start', default=False,
                 action='store_true', help="Don't start processes, just" +\
                 " report whether processes that would be started are running")
    p.add_option('-d', '--show-details', dest='details', default=True,
                 action='store_true', help='display additional details')
    p.add_option('-n', '--no-show-details', dest='details',
                 action='store_false', help='display additional details')
    p.add_option('-p', '--persistent', dest='persistent', default=False,
                 action='store_true', help='keep going until stopped')
    
    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)
    
    if opts.no_start:
        while True:
            printDetails(getProcesses(), opts.details)
            time.sleep(3.0)
            if not opts.persistent:
                break
    else:
        while True:
            processes = getProcesses()
            printDetails(processes, opts.details)
            startInactive(processes)
            time.sleep(3.0)
            if not opts.persistent:
                break

