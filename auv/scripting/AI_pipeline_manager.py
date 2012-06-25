#!/usr/bin/env python2.7
from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import messaging
import cauv.pipeline
import utils.event as event

import threading, os, os.path, cPickle, time
import traceback, argparse, Queue, collections
import re
from glob import glob
from datetime import datetime

class pipelineManager(aiProcess):
    def __init__(self, opts):
        aiProcess.__init__(self, "pl_manager")
        #TODO make options actually work
        self.disable_gui = opts.disable_gui 
        self.freeze = opts.freeze_pls
        #TODO restore pipelines requested by scripts
        if opts.restore:
            self.requests = {'script':{}, 'other':{}}
        else:
            self.requests = {'script':{}, 'other':{}}

        self.detector_requests = []
        
        self.pipelines = {}
        self.running_pls = {}
        
        self.load_pl_data()
        
        #start receiving messages
        self._register()

    def load_pl_data(self):
        """
        try and load pipelines
        """

        #make sure our folder for temporary files exists
        try:
            os.mkdir(os.path.join('pipelines','temp'))
        except OSError:
            pass
        
        #scan the pipelines
        pipeline_files = glob(os.path.join('pipelines', '*.pipe')) + \
                         glob(os.path.join('pipelines', 'temp', '*.pipe'))

        pipeline_names = [os.path.basename(x).rsplit('.',1)[0] for x in pipeline_files]
        pipeline_groups = collections.defaultdict(list) #pipeline main name: filenames
        for name, filename in zip(pipeline_names, pipeline_files):
            if name.startswith('temp__'):
                warning('Found temporary pipelines in file %s.pipe, it is recommended these are sorted and deleted' %(name,))
                pipeline_groups[name[6:]].append(filename)
            else:
                pipeline_groups[name].append(filename)

        for name, filenames in pipeline_groups.iteritems():
            filename = max(filenames, key=os.path.getmtime)
            self.add_pipeline(name, filename)

    def add_pipeline(self, name, filename):
        #check whether we've already loaded this pipeline
        if name in self.pipelines:
            warning('Multiple pipelines with same name (%s).' %(name))
        #load pipeline
        try:
            pl_state = cauv.pipeline.Model.loadFile(open(filename))
        except:
            error('Error loading pipeline %s (%s)' %(name, filename))
            error(traceback.format_exc().encode('ascii','ignore'))
            return
        self.pipelines[name] = pl_state
        
    @external_function
    @event.event_func
    def force_save(self, pipelines=None, temporary=False):
        self.eval_state()

    @external_function
    @event.event_func
    def request_pl(self, requestor_type, requestor_name, requested_pl):
        #VERY IMPORTANT: make sure requestor_name is same as process_name
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
        self.eval_state()
        #confirm request (atm only scripts)
        if requestor_type == 'script':
            getattr(self.ai, requestor_name).confirm_pl_request()

    @external_function
    @event.event_func
    def drop_pl(self, requestor_type, requestor_name, requested_pl):
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
            self.eval_state()

    @external_function
    @event.event_func
    def drop_all_pls(self, requestor_type, requestor_name):
        try:
            self.requests[requestor_type][requestor_name] = []
            self.eval_state()
        except KeyError:
            error('Could not drop pls, requestor type %s invalid' %(requestor_type,))
    @external_function
    def drop_task_pls(self, task_id):
        self.requests['script'][task_id] = []
        self.eval_state()

    @external_function
    @event.event_func
    def set_detector_pls(self, req_list):
        if set(req_list) != set(self.detector_requests):
            self.detector_requests = req_list
            debug("Setting detector requests to {}".format(self.detector_requests))
            self.eval_state()

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
        debug("State: running_pls: {}, to_remove: {}, to add: {}".format(
               files_in_use, to_remove, to_add))
        for reqname in to_remove:
            pl = self.running_pls.pop(reqname) #remove from running_list
            try:
                pl.save('pipelines/temp/temp__'+reqname+'.pipe')
            except Exception:
                error('Error saving pipeline %s' %(reqname))
                error(traceback.format_exc().encode('ascii','ignore'))
            pl.clear()
        #4
        for reqname in to_add:
            pl = cauv.pipeline.Model(self.node, 'ai/'+reqname)
            pl.set(self.pipelines[reqname])
            self.running_pls[reqname] = pl

    if False:
        def run(self):
            #this should probably be replaced with multiple pipelines
            while self.running:
                for x in range(5):
                    try:
                        req = self.request_queue.get(block=True, timeout=0.25)
                    except Queue.Empty:
                        break
                    try:
                        req[0](**req[1])
                    except Exception:
                        error('Exception while trying to manage pipeline: ')
                        error(traceback.format_exc().encode('ascii','ignore'))
                else:
                    if self.request_queue.qsize()>5:
                        warning('Pipeline manager request queue is large, there may be delays')
                if self.reeval:
                    self.reeval = False
                    self.eval_state()
                    #self.state['requests'] = self.requests
                    #self.state.sync()
    

if __name__ == '__main__':
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
    pm = pipelineManager(opts)
    try:
        pm.run()
    finally:
        pm.die()
