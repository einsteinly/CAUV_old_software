#!/usr/bin/env python2.7
from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import messaging
import cauv.pipeline

import threading, os, os.path, cPickle, time, traceback, pickle, argparse, Queue, subprocess
from glob import glob
from datetime import datetime

class pipelineManager(aiProcess):
    def __init__(self, *args, **kwargs):
        aiProcess.__init__(self, *args, **kwargs)
        #make sure our folder for temporary files exists
        try:
            os.mkdir(os.path.join('pipelines','temp'))
        except OSError:
            pass
        
        #TODO make options actually work
        self.disable_gui = kwargs['disable_gui'] if 'disable_gui' in kwargs else False
        self.request_queue = Queue.Queue()
        self.freeze = False
        if 'freeze_pls' in kwargs:
            if kwargs['freeze_pls']:
                self.freeze = True
                
        #TODO restore pipelines requested by scripts
        try:
            if kwargs['restore']:
                pass
            else:
                self.requests = {'script':{}, 'other':{}}
        except KeyError:
            self.requests = {'script':{}, 'other':{}} #script_name: pl_names
        self.detector_requests = []
        
        self.pipelines = {}
        self.running_pls = {}
        
        self.load_pl_data()
        self.reeval = False
        
    def load_pl_data(self):
        """
        try and load pipelines
        """
        #scan the pipelines
        pipeline_names = [x[10:-5] for x in glob(os.path.join('pipelines', '*.pipe'))]
        pipeline_names.extend([x[10:-5] for x in glob(os.path.join('pipelines','temp','*.pipe'))])#we'll drop the 'pipelines/' prefix and '.pipe'
        pipeline_groups = {} #pipeline main name: filenames
        for name in pipeline_names:
            if name.startswith('temp__'):
                warning('Found temporary pipelines in file %s.pipe, it is recommended these are sorted and deleted' %(name,))
                try:
                    pipeline_groups[name[6:]].append(name)
                except KeyError:
                    pipeline_groups[name[6:]] = [name]
            else:
                try:
                    pipeline_groups[name].append(name)
                except KeyError:
                    pipeline_groups[name] = [name]
        for name, filenames in pipeline_groups.iteritems():
            filename = max(filenames, key=lambda x: os.path.getmtime('pipelines/'+x+'.pipe'))
            self.add_pipeline(name, filename)
    def add_pipeline(self, name, filename):
        #check whether we've already loaded this pipeline
        if name in self.pipelines:
            warning('Multiple pipelines with same name (%s).' %(name))
        #load pipeline
        try:
            pl_state = cauv.pipeline.Model.loadFile(open(os.path.join('pipelines', filename)+'.pipe'))
        except:
            error('Error loading pipeline %s' %(filename,))
            traceback.print_exc()
            return
        self.pipelines[name] = pl_state
        
    @external_function
    def force_save(self, pipelines=None, temporary=False):
        self.reeval = True
    @external_function
    def request_pl(self, requestor_type, requestor_name, requested_pl):
        #VERY IMPORTANT: make sure requestor_name is same as process_name
        #since the python thread cannot be interrupted multiple times it seems, and needs to be interrupted to set up the pipeline, request must be handled in the main process
        self.request_queue.put(('setup_pl',{'requestor_type':requestor_type, 'requestor_name':requestor_name, 'requested_pl':requested_pl}))
    @external_function
    def drop_pl(self, requestor_type, requestor_name, requested_pl):
        self.request_queue.put(('unsetup_pl',{'requestor_type':requestor_type, 'requestor_name':requestor_name, 'requested_pl':requested_pl}))
    @external_function
    def drop_all_pls(self, requestor_type, requestor_name):
        self.request_queue.put(('unsetup_all',{'requestor_type':requestor_type, 'requestor_name':requestor_name}))
    @external_function
    def drop_task_pls(self, task_id):
        self.request_queue.put(('unsetup_task',{'task_id':task_id}))
    @external_function
    def set_detector_pls(self, req_list):
        self.request_queue.put(('setup_detectors',{'req_list':req_list}))
    def setup_pl(self, requestor_type, requestor_name, requested_pl):
        if not requestor_type in self.requests:
            error('Invalid requestor type %s, named %s' %(requestor_type, requestor_name))
            return
        if not requested_pl in self.pipelines:
            error('Non-existant pipeline %s requested' %(requested_pl,))
            return
            #update request info
        if requestor_name in self.requests[requestor_type]:
            self.requests[requestor_type][requestor_name].append(requested_pl)
        else:
            self.requests[requestor_type][requestor_name] = [requested_pl]
        #confirm request (atm only scripts)
        if requestor_type == 'script':
            getattr(self.ai, requestor_name).confirm_pl_request()
        self.reeval = True
    def unsetup_pl(self, requestor_type, requestor_name, requested_pl):
        if not requestor_type in self.requests:
            error('Invalid type in requested drop pl')
        elif not requestor_name in self.requests[requestor_type]:
            error('Requestor does not have any pipelines to drop')
        elif not requested_pl in self.requests[requestor_type][requestor_name]:
            error('Requestor has not requested this pipeline %s' %(requested_pl,))
        elif not requested_pl in self.pipelines:
            warning('Could not drop request %s, may have left things in the pipeline.' %(requested_pl, ))
        else:
            self.requests[requestor_type][requestor_name].remove(requested_pl)
        self.reeval = True
    def unsetup_all(self, requestor_type, requestor_name):
        try:
            self.requests[requestor_type][requestor_name] = []
        except KeyError:
            error('Could not drop pls, requestor type %s invalid' %(requestor_type,))
        self.reeval = True
    def unsetup_task(self, task_id):
        self.requests['script'][task_id] = []
        self.reeval = True
    def setup_detectors(self, req_list):
        if set(req_list) != set(self.detector_requests):
            self.detector_requests = req_list
            self.reeval = True
            
            
    def eval_state(self):
        """
        TODO allow gui disabling, freezing
        1) check list of currently running pl files against requested
        2) clear unneeded pipelines, saving to temp
        3) add any missing requested pl files
        """
        #1
        files_requested = set(self.detector_requests)
        for requestors in self.requests.itervalues():
            for requests in requestors.itervalues():
                files_requested.update(requests)
        files_in_use = set(self.running_pls.keys())
        to_remove = files_in_use.difference(files_requested)
        to_add = files_requested.difference(files_in_use)
        #2
        for reqname in to_remove:
            pl = self.running_pls.pop(reqname) #remove from running_list
            try:
                pl.save('pipelines/temp/temp__'+name+'.pipe')
            except Exception:
                error('Error saving pipeline %s' %(reqname))
                traceback.print_exc()
            pl.clear()
        #4
        for reqname in to_add:
            pl = cauv.pipeline.Model(self.node, 'ai/'+reqname)
            pl.set(self.pipelines[reqname])
            self.running_pls[reqname] = pl
    def run(self):
        #this should probably be replaced with multiple pipelines
        while True:
            for x in range(5):
                try:
                    req = self.request_queue.get(block=True, timeout=0.25)
                except Queue.Empty:
                    break
                try:
                    getattr(self,(req[0]))(**req[1])
                except Exception:
                    error('Exception while trying to setup pipeline caused by '+str(req[0])+str(req[1]))
                    traceback.print_exc()
            else:
                if self.request_queue.qsize()>5:
                    warning('Pipeline manager request queue is large, there may be delays')
            if self.reeval:
                self.reeval = False
                self.eval_state()
                #self.state['requests'] = self.requests
                #self.state.sync()
    

# These are the BSD ones
Signal_Names = {
    1  : "SIGHUP",
    2  : "SIGINT",
    3  : "SIGQUIT",
    4  : "SIGILL",
    5  : "SIGTRAP",
    6  : "SIGABRT",
    7  : "SIGEMT",
    8  : "SIGFPE",
    9  : "SIGKILL",
    10 : "SIGBUS",
    11 : "SIGSEGV",
    12 : "SIGSYS",
    13 : "SIGPIPE",
    14 : "SIGALRM",
    15 : "SIGTERM",
    16 : "SIGURG",
    17 : "SIGSTOP",
    18 : "SIGTSTP",
    19 : "SIGCONT",
    20 : "SIGCHLD",
    21 : "SIGTTIN",
    22 : "SIGTTOU",
    23 : "SIGIO",
    24 : "SIGXCPU",
    25 : "SIGXFSZ",
    26 : "SIGVTALRM",
    27 : "SIGPROF",
    28 : "SIGWINCH",
    29 : "SIGINFO",
    30 : "SIGUSR1",
    31 : "SIGUSR2"
}

import traceback
def sigHandler(signum, frame):
    error('Signal %s (%d) in %s' % (Signal_Names[signum], signum, ''.join(traceback.format_stack(frame))))
    raise Exception('caught signal')
        
if __name__ == '__main__':
    # temp debug code, try to work out where the KeyboardInterrupt is coming
    # from:
    #import signal
    """
    for i in xrange(0, signal.NSIG):
        try: 
            signal.signal(i, sigHandler);
            debug('installed signal handler for %d' % i)
        except ValueError:
            pass
        except RuntimeError:
            pass
    """
    p = argparse.ArgumentParser()
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_argument('-g', '--disable_gui', dest='disable_gui', default=False,
                 action='store_true', help="disable/ignore gui output nodes")
    p.add_argument('--reset_pls', dest='reset_pls', default=False,
                 action='store_true', help="reset pipelines to those stored in /pipelines")
    p.add_argument('--freeze_pls', dest='freeze_pls', default=False,
                 action='store_true', help="ignore changes to the pipeline")
    opts, args = p.parse_known_args()
    pm = pipelineManager('pl_manager')
    try:
        pm.run()
    finally:
        pm.die()
