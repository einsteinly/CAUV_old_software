import collections
import errno
import imp
import os
import pwd
import resource
import shlex
import subprocess
import sys
import time
import watchfuncs

import utils.multitasking
from cauv.debug import debug, info, warning, error

class SessionNotFound(Exception):
    pass

def load_session(filename):
    for suffix, mode, type in imp.get_suffixes():
        if filename.endswith(suffix):
            with open(filename, mode) as session_file:
                return imp.load_module("session", session_file, "session.py", (suffix, mode, type))
    raise SessionNotFound()

class WatchProcess:
    def __init__(self, process, settings):
        self.p = process
        self.pid = None
        self.restart = True
        self.settings = settings

    def exec_proc(self):
        format_dict = {'SDIR': os.path.abspath(self.settings.script_dir), "BDIR": os.path.abspath(self.settings.bin_dir), "D": ''}
        command = self.p.cmds[0].format(**format_dict)
        work_dir = self.p.work_dir.format(**format_dict)
        os.chdir(work_dir)

        if self.p.user is None:
            username = self.settings.user
        else:
            username = self.p.user
        debug('running \'{}\' in directory {} as {}'.format(command, work_dir, username))
        try:
            uid, gid = pwd.getpwnam(username)[2:4]
            if sys.platform == 'darwin':
                # no such thing as a saved id...
                os.setgid(gid)
                os.setuid(uid)
            else:
                os.setresgid(gid,gid,gid)
                os.setresuid(uid,uid,uid)
        except OSError as e:
            if e.errno == errno.EPERM:
                warning("Can't change user. Running as current user instead")
            else:
                raise

        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())

        argv = shlex.split(command)
        os.execlp(argv[0], *argv)

    def tick(self):
        if self.pid is not None:
            try:
                os.kill(self.pid, 0)
            except OSError as e:
                if e.errno == errno.ESRCH:
                    self.pid = None
            if self.pid is None:
                warning("Process {} died!".format(self.p.name))
                self.restart = self.p.death_callback(self.p)
        if self.pid is None:
            self.pid = self.p.pid_func()
            if self.pid is not None:
                info("found PID {} for process {}".format(self.pid, self.p.name))
        if self.pid is None and self.restart:
            self.pid = utils.multitasking.spawnDaemon(self.exec_proc)

    def kill(self, sig):
        if self.pid is None:
            self.pid = self.p.pid_func()
        if self.pid is not None:
            info("Killing PID {} ({})".format(self.pid, self.p.name))
            try:
                os.kill(self.pid, sig)
            except OSError as e:
                error("Could not kill PID: {}".format(e))
        else:
            warning("Could not find PID for {}".format(self.p.name))

def setup_core_dumps():
    core_pattern = "/tmp/corefiles/%e.%d.%t.%p"
    try:
        try:
            os.mkdir("/tmp/corefiles", 0777)
        except OSError as e:
            if e.errno != errno.EEXIST:
                error("Could not create corefile directory")
                raise
        with open("/proc/sys/kernel/core_pattern", "r") as pattern_file:
            if not pattern_file.read().startswith(core_pattern):
                with open("/proc/sys/kernel/core_pattern", "w") as pattern_w_file:
                    pattern_w_file.write(core_pattern)
        soft, hard = resource.getrlimit(resource.RLIMIT_CORE)
        resource.setrlimit(resource.RLIMIT_CORE, (min(hard, 10*1024*1024), hard))
    except IOError as e:
        if e.errno == errno.EACCES:
            error("Could not change core pattern: not running as root?")
        elif e.errno == errno.ENOENT:
            error("Could not find core pattern file. Probably using a Mac...")
        else:
            raise
    
Settings = collections.namedtuple('Settings', ['user', 'script_dir', 'bin_dir'])

class Watcher:
    def __init__(self, processes = None, core_dumps = False, log_dir = None,
            default_user = None, script_dir = '', bin_dir = ''):
        if core_dumps:
            setup_core_dumps()
        if processes is None:
            processes = []
        if log_dir:
            os.environ["CAUV_LOG_DIR"] = log_dir

        self.settings = Settings(default_user, script_dir, bin_dir)

        self.processes = [WatchProcess(p, self.settings) for p in processes]

    def monitor(self, tick_time = 1):
        while True:
            for p in self.processes:
                p.tick()
            time.sleep(tick_time)

    def kill(self, signal):
        for p in self.processes:
            p.kill(signal)

