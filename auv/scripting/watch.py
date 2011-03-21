import traceback
import psutil
import optparse

from cauv.debug import debug, info, warning, error


class CAUVTask:
    def __init__(self, command, restart=True):
        self.command = command
        self.restart = restart
        self.PID = None
    def isRunning(self):
        return self.PID != None

processes_to_start = [
        CAUVTask('./run.sh remote.py', True),
        CAUVTask('./run.sh logger.py', False)
]

def getPIDs():
    pids = psutil.get_pid_list()
    # find the pids we're interested in
    for pid in pids:
        try:
            p = psutil.Process(pid)
            debug('%s' % (p.exe))
            debug('%s' % (p.cmdline))
            debug('%s' % (p.name))
        except Exception, e:
            warning('%s\n%s' % (e, traceback.format_exc()))


if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-N', '--no-start', dest='no_start', default=False,
                 action='store_true', help="Don't start processes, just" +\
                 " report whether processes that would be started are running")
    p.add_option('-d', '--show-details', dest='details', default=False,
                 action='store_true', help='display additional details')
    
    opts, args = p.parse_args()

    if len(args) > 0:
        print 'this program takes no arguments'
        exit(1)
    
    if opts.no_start:
        getPIDs()

