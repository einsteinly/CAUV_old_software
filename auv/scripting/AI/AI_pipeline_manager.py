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
        aiProcess.__init__(self, "pl_manager", opts.manager_id)
        #TODO make options actually work
        self.disable_gui = opts.disable_gui 
        self.freeze = opts.freeze_pls
        #TODO restore pipelines requested by scripts
        self.hold_pls = opts.hold_pls
        self.load_temp = opts.load_temp
        self.simulation = opts.simulation

        self.detector_requests = []
        self.task_requests = {}
        
        self.pipelines = {}
        self.created_pls = {}
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
                if self.load_temp:
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
            #check node types
            for node_id, node in pl_state.nodes.iteritems():
                if node.type in (messaging.NodeType.FileInput, messaging.NodeType.DirectCameraInput):
                    warning("Pipeline contains a testing input, this probably wasn't intended, please fix !!!")
                elif node.type == messaging.NodeType.NetInput:
                    warning("Pipeline contains a testing input, this probably wasn't intended, please fix !!!")
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
    def set_detector_pls(self, req_list):
        if set(req_list) != set(self.detector_requests):
            self.detector_requests = req_list
            debug("Setting detector requests to {}".format(self.detector_requests))
            self.eval_state()

    @external_function
    @event.event_func
    def set_task_pls(self, task_id, req_list):
        self.task_requests[task_id] = req_list
        self.eval_state()
            
    @external_function
    @event.event_func
    def start_task_pls(self, task_id):
        for pl_name in self.task_requests[task_id]:
            self.created_pls[pl_name].play()
            
    @external_function
    @event.event_func
    def stop_task_pls(self, task_id):
        for pl_name in self.task_requests[task_id]:
            if not pl_name in self.detector_requests:
                self.created_pls[pl_name].pause()

    def eval_state(self):
        """
        TODO allow gui disabling, freezing
        1) check list of currently running pl files against requested
        2) clear unneeded pipelines, saving to temp
        3) add any missing requested pl files
        """
        #1
        files_requested = set(self.detector_requests)
        for task_id, pl_names in self.task_requests.iteritems():
            files_requested.update(set(pl_names))
        files_in_use = set(self.created_pls.keys())
        to_remove = files_in_use.difference(files_requested)
        to_add = files_requested.difference(files_in_use)
        #2
        debug("State: running_pls: {}, to_remove: {}, to add: {}".format(
               files_in_use, to_remove, to_add))
        for reqname in to_remove:
            try:
                self.running_pls.pop(reqname) #remove from running_list
            except KeyError:
                pass
            try:
                pl = self.created_pls.pop(reqname) #remove from running_list
            except KeyError:
                error('Tried to remove non-existant pipeline %s' %reqname)
                continue
            try:
                pl.save('pipelines/temp/temp__'+reqname+'.pipe')
            except Exception:
                error('Error saving pipeline %s' %(reqname))
                error(traceback.format_exc().encode('ascii','ignore'))
            if not self.hold_pls:
                pl.clear()
        #4
        for reqname in to_add:
            pl = cauv.pipeline.Model(self.node, 'ai/'+reqname)
            pl.set(self.pipelines[reqname])
            self.created_pls[reqname] = pl
            if not reqname in self.detector_requests:
                pl.pause()
            else:
                self.running_pls[reqname] = pl

        """
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
        """

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-r', '--restore', dest='restore', default=False,
                 action='store_true', help="try and resume from last saved state")
    p.add_argument('-g', '--disable_gui', dest='disable_gui', default=False,
                 action='store_true', help="disable/ignore gui output nodes")
    p.add_argument('--reset_pls', dest='reset_pls', default=False,
                 action='store_true', help="reset pipelines to those stored in /pipelines")
    p.add_argument('--hold_pls', dest='hold_pls', default=False,
                 action='store_true', help="Don't clear pipelines when they've been finished with.")
    p.add_argument('--load_temp', dest='load_temp', default=False,
                 action='store_true', help="Load any temporary files created.")
    p.add_argument('--freeze_pls', dest='freeze_pls', default=False,
                 action='store_true', help="ignore changes to the pipeline")
    p.add_argument('--simulation', dest='simulation', default=False,
                 action='store_true', help="replace relevent nodes with net input")
    p.add_argument('-M', '--manager_id', dest='manager_id', default='',
                 action='store', help="id of relevent ai manager")
    opts, args = p.parse_known_args()
    pm = pipelineManager(opts)
    try:
        pm.run()
    finally:
        pm.die()
