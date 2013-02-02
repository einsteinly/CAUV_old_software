import collections
import subprocess
import tempfile
import shutil
import os.path
import os
import re
import unittest
import cauv.node as node

LogLine = collections.namedtuple("LogLine", ["timestamp", "thread", "level", "body"])

def grep_log(log, text_match):
    m = re.compile(text_match)
    return [l for l in log if m.match(l.body)]

def grep_log_level(log, level):
    pass

class TempProcessEnv(object):
    def __init__(self, create_node = False):
        self.create_node = create_node

    def start_process(self, args):
        devnull = open(os.devnull, "rw")
        self.processes.add(subprocess.Popen(
            args,
            stdin = devnull,
            stdout = devnull,
            stderr = devnull,
            close_fds = True
        ))

    def get_log(self, filename):
        log_dir = os.path.join(self.dirname, filename)
        log = []
        with open(log_dir) as logf:
            log_str = re.sub("\x1b[^m]*m", "", logf.read())
            for line in log_str.split("\n[")[1:]:
                leader, body = line.split(": ",1)
                timestamp, thread, level = leader.split(" ")
                thread = thread[:1]
                log.append(LogLine(timestamp, thread, level, body))
        return log

    def __enter__(self):
        self.dirname = tempfile.mkdtemp()
        self.processes = set()
        log_dir = os.path.join(self.dirname, "log")
        os.mkdir(log_dir)
        os.mkdir(os.path.join(self.dirname, "test"))
        #bleh, this is a bit ugly, but can't really modify these options easily
        #otherwise....
        os.environ.update({
            "CAUV_IPC_DIR": self.dirname,
            "CAUV_VEHICLE_NAME": "test",
            "CAUV_LOG_DIR": log_dir,
        })
        if self.create_node:
            self.node = node.Node("test")
        return self

    def __exit__(self, type, value, tb):
        for process in self.processes:
            process.kill()
            process.wait()
        shutil.rmtree(self.dirname)

class ProcessTestCase(unittest.TestCase):
    def __init__(self, *args, **kargs):
        super(ProcessTestCase, self).__init__(*args, **kargs)
        self.relevant_log = None
        self.longMessage = True

    #naughty, overriding internal methods...
    def _formatMessage(self, msg, standardMessage):
        if self.relevant_log is not None:
            if msg is None:
                msg = ''
            msg += '\n Log file from process: \n'
            msg += '\n'.join([l.body for l in self.relevant_log])
        return super(ProcessTestCase, self)._formatMessage(msg, standardMessage)
