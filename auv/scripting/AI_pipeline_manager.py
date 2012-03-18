from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import pipeline, messaging

import threading, os.path, cPickle, time, traceback, pickle, argparse, Queue, subprocess
from glob import glob
from datetime import datetime

IMG_PIPELINE = 'cauv-img-pipeline'
IMG_PIPELINE_START_TIME = 2

#lets modify pipeline.Model class a bit so its a bit more flexible
class NewModel(pipeline.Model):
    """
    Major features as compared to the original pipeline model:
     - Runs it's own image pipeline, complete with dealing with crashes
     - Keeps nodes labeled the same as they were handed to it, and supports relabeling nodes
    """
    def __init__(self, node, pipeline_name, *args, **kwargs):
        self.img_pipeline_name = pipeline_name
        self.img_pipeline = subprocess.Popen([IMG_PIPELINE, pipeline_name])
        try:
            pipeline.Model.__init__(self, node, pipeline_name, *args, **kwargs)
        except RuntimeError as e:
            error('Communication problems while initialising model, giving up.')
            traceback.print_exc()
            raise e
        self.manager2pl = {}#node ids here: node ids in pl
        self.pl2manager = {}
        self.nodes = {}#record of what we think is in the pipeline
        self.temp_number = 0#labeling for any new user created nodes
        self.bad_ids #list of node ids that cause crashes
    def get(self, timeout=3.0): #needs nnid to know where new nodes are out of the way
        try:
            state = self.catch_pl_errors(pipeline.Model.get, args=(self, timeout))
        except RuntimeError:
            error('Communication problems while trying to get the state of the pipeline, giving up after too many attempts to reinitialise.')
            traceback.print_exc()
            raise e
        renumbered_state = pipeline.State()
        new_nodes = []
        #check for new nodes
        for node_id in state.nodes.keys():
            if not node_id in self.pl2manager:
                new_nodes.append('temp'+str(self.temp_number))
                self.manager2pl['temp'+str(self.temp_number)] = node_id
                self.pl2manager[node_id] = 'temp'+str(self.temp_number)
                self.temp_number += 1
        #now move state over to renumbered_state
        for node_id, node in state.nodes.items():
            for input, inarc in node.inarcs.items():
                if inarc[0] == 0:
                    #get rid of silly 0s
                    node.inarcs.pop(input)
                    continue
                node.inarcs[input] = (self.pl2manager[inarc[0]], inarc[1])
            for output, outarcs in node.outarcs.items():
                node.outarcs[output] = [(self.pl2manager[outarc[0]], outarc[1]) for outarc in outarcs]
            renumbered_state.nodes[self.pl2manager[node_id]] = node
        #update internal state
        self.nodes = renumbered_state.nodes
        return renumbered_state, new_nodes
    def reassign(self, relabel):
        new_manager2pl = {}
        new_pl2manager = {}
        for old_id in self.manager2pl:
            if old_id in relabel:
                new_pl2manager[self.manager2pl[old_id]] = relabel[old_id]
                new_manager2pl[relabel[old_id]] = self.manager2pl[old_id]
            else:
                new_pl2manager[self.manager2pl[old_id]] = old_id
                new_manager2pl[old_id] = self.manager2pl[old_id]
        self.manager2pl = new_manager2pl
        self.pl2manager = new_pl2manager
    def set(self, state):
        raise NotImplementedError
    def add(self, nodes, timeout=5.0):
        for node_id, node in nodes.items():
            if node_id in self.manager2pl:
                warning('Node already added, skipping')
                continue
            if node_id in self.bad_ids:
                warning('Node is marked as bad, ignoring')
                continue
            try:
                new_node_id = self.catch_pl_errors(self.addSynchronous, args=(node.type, timeout), limit=1)
            except RuntimeError:
                error("Couldn't add node %d, marking as bad and skipping" %(new_node_id))
                self.bad_ids.append(node_id)
                continue
            self.manager2pl[node_id] = new_node_id
            self.pl2manager[new_node_id] = node_id
            self.nodes[node_id]=node
            bad_params = []
            for param, value in node.params.items():
                try:
                    self.catch_pl_errors(self.setParameterSynchronous, args=(new_node_id, param, value, timeout), limit=1)
                except RuntimeError:
                    error("Couldn't set parameter: "+str((param,value)))
                    #remove parameter to avoid future errors
                    bad_params.append(param)
            for param in bad_params:
                node.params.pop(param)
                    
        #now all added, add arcs
        for node_id, node in nodes.items():
            bad_arcs = []
            for input, (from_node_id, from_node_field) in node.inarcs.items():
                try:
                    try:
                        self.catch_pl_errors(self.addArcSynchronous, args=(self.manager2pl[from_node_id], from_node_field, self.manager2pl[node_id], input, timeout), limit=1)
                    except KeyError:
                        error('Could not add arc from %d to %d, a node does not exist.' %(from_node_id, node_id))
                except RuntimeError:
                    error("Couldn't add arc from %d (%s) to %d (%s) (%d to %d in pipeline), removing arc" %(from_node_id, from_node_field, node_id, input, self.manager2pl[from_node_id], self.manager2pl[node_id]))
                    bad_arcs.append(input)
            for input in bad_arcs:
                node.inarcs.pop(input)
                #This could cause errors if we ever use outarcs
    def remove(self, nodes, timeout=5.0):
        for node_id in nodes:
            try:
                try:
                    self.catch_pl_errors(self.removeSynchronous, args=(self.manager2pl[node_id], timeout))
                except RuntimeError:
                    error("Couldn't remove node: removing from state and restarting pipeline to flush badness")
            except KeyError:
                error("Couldn't remove node: no record of node exists")
            #want to remove nodes regardless otherwise wont be able to add them again
            self.pl2manager.pop(self.manager2pl[node_id])
            self.manager2pl.pop(node_id)
            self.nodes.pop(node_id)
            self.restart_pipeline()
    def restart_pipeline(self):
        self.img_pipeline.kill()
        self.img_pipeline = subprocess.Popen([IMG_PIPELINE, pipeline_name])
        time.sleep(IMG_PIPELINE_START_TIME)
        #copy nodes, then reset
        nodes = self.nodes
        self.nodes = {}
        self.add(nodes)
    def catch_pl_errors(function, args=[], kwargs={}, limit=3):
        for attempt in range(limit):
            try:
                return function(*args, **kwargs)
            except RuntimeError:
                pass
            #check that pipeline hasn't died
            if not self.img_pipeline.poll():
                self.restart_pipeline()
                continue
            #check that the pipeline is still responding by getting state
            try:
                pipeline.Model.get(3.0)
            except RuntimeError:
                self.restart_pipeline()
                continue
            #appears to be working fine, probably a bad function call, so don't bother retrying
            break
        raise RuntimeError('Pipeline seems fine, bad function call?')

class Pipeline():
    def __init__(self, nodes):
        self.nodes = nodes #node_id: node
    
class PipelinesSet():
    def __init__(self):
        self.request2filename = {}
        self.pipelines = {}
        self.node_id2pipeline = {} #for fast lookups
        self.nnid = 0
    def add_pipeline(self, name, filename):
        #check whether we've already loaded this pipeline
        if filename in self.pipelines:
            self.request2filename[name] = filename
            return
        #load pipeline
        try:
            pl_state = cPickle.load(open(os.path.join('pipelines', pl_name, '.pipe')))
        except:
            error('Error loading pipeline %s' %(filename,))
            traceback.print_exc()
            return
        old_id2new_id = {}
        nodes = {}
        #relabel nodes
        for node_id, node in pl_state.nodes.items():
            old_id2new_id[node_id] = self.nnid
            nodes[self.nnid] = node
            self.node_id2pipeline[self.nnid]=filename
            self.nnid += 1
        #relabel inarcs, remove troublesome outputs and zeroes
        for node in nodes.itervalues():
            to_remove = []
            for input, inarc in node.inarcs.items():
                if inarc[0] == 0:
                    to_remove.append(input)
                else:
                    inarc[0] = old_id2new_id[inarc[0]]
            node.outarcs = None
        self.pipelines[filename] = Pipeline(nodes)
        self.request2filename[name] = filename
    def merge_and_update(self, merged_pls):
        for pipelines, nodes in merged_pls:
            #1) merge pipelines
            if len(pipelines)>1:
                #first need generate a name for our merge
                pl_names = []
                for pl in pipelines:
                    if pl.startswith(merged):
                        pl_names.extend(pl.split('__')[1:-1])
                    else:
                        pl_names.append(pl)
                    #remove old pipelines
                    self.pl_data.pipelines.pop(pl)
                pipeline = 'merged__'+'__'.join(pl_names)+'__all'
                #update requests to map to this new pl
                for pl in pipelines:
                    if pl.startswith(merged):
                        for name in pl.split('__')[1:-1]:
                            self.pl_data.request2filename[name] = pipeline
                    else:
                        self.pl_data.request2filename[pl] = pipeline
            else:
                pipeline = pipelines[0]
            #2) set pipeline to have new nodes
            self.pl_data.pipelines = Pipeline(nodes)
    def get_relabeling(new_node_ids):
        self.nnid += len(new_node_ids)
        return dict([(node_id, self.nnid-index-1) for index, node_id in enumerate(new_node_ids)])

class pipelineManager(aiProcess):
    def _setup(self, *args, **kwargs):
        #TODO make options actuall work
        self.disable_gui = kwargs['disable_gui'] if 'disable_gui' in kwargs else False
        self.request_queue = Queue.Queue()
        self.freeze = False
        if 'freeze_pls' in kwargs:
            if kwargs['freeze_pls']:
                self.freeze = True
        #TODO state saving
        try:
            if kwargs['restore']:
                pass
            else:
                self.requests = {'script':{}, 'other':{}}
        except KeyError:
            self.requests = {'script':{}, 'other':{}} #script_name: pl_names
        self.detector_requests = []
        self.pl_data = PipelinesSet()
        self.load_pl_data()
        #this should probably be replaced with multiple pipelines
        self._pl = NewModel(self.node, 'ai')
        self.pl_state = pipeline.State()
        self.reeval = False
    def load_pl_data(self):
        """
        try and load pipelines
        syntax for merged pipelines
        merged__PLNAME1__PLNAME2...__1/2/../all (so if multiple pls get merged, we still get what we want)
        The strategy is 1) look for all pipelines (except merged__...__1/2.../n since these are only part pipelines)
        2)group all pipelines with the same name 3)choose most recently modified pl
        """
        #scan the pipelines
        pipeline_names = [x[10:-5] for x in glob(os.path.join('pipelines', '*.pipe'))] #we'll drop the 'pipelines/' prefix and '.pipe'
        pipeline_groups = {} #pipeline main name: filenames
        for name in pipeline_names:
            if name.startswith('merged'):
                if not name.endswith('all'):
                    #so is only a partial pl, so ignore
                    continue
                warning('Found merged pipelines in file %s.pipe, it is recommended these are split and the merged file deleted' %(name,))
                names = name.split('__')[1:-1]
                for pl_name in names:
                    try:
                        pipeline_groups[pl_name].append(name)
                    except KeyError:
                        pipeline_groups[pl_name] = [name]
            else:
                try:
                    pipeline_groups[name].append(name)
                except KeyError:
                    pipeline_groups[name] = [name]
        for name, filenames in pipeline_groups.iteritems():
            filename = max(filenames, key=lambda x: os.path.getmtime('pipelines/'+x+'.pipe'))
            self.pl_data.add_pipeline(name, filename)
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
    def drop_all_pl(self, requestor_type, requestor_name):
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
        if not requested_pl in self.pl_data.pipelines:
            error('Non-existant pipeline %s requested' %(requested_pl,))
            return
            #update request info
        if requestor_name in self.requests[requestor_type]:
            self.requests[requestor_type][requestor_name].append(requested_pl)
        else:
            self.requests[requestor_type][requestor_name] = [requested_pl]
        #confirm setup (atm only scripts)
        if requestor_type == 'script':
            getattr(self.ai, requestor_name).confirm_pl_response()
        self.reeval = True
    def unsetup_pl(self, requestor_type, requestor_name, requested_pl):
        if not requestor_type in self.requests:
            error('Invalid type in requested drop pl')
        elif not requestor_name in self.requests[requestor_type]:
            error('Requestor does not have any pipelines to drop')
        elif not requested_pl in self.requests[requestor_type][requestor_name]:
            error('Requestor has not requested this pipeline %s' %(requested_pl,))
        elif not requested_pl in self.pl_data.pipelines:
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
        TODO allow gui disabling
        1) get the current state (also returns a list of new nodes)
            check that state isn't empty, if it is (andpl_state isn't) probably crashed, so don't do checking
        IF NOT FROZEN:
        2) change all the temporary nodes to proper nodes
        3) modify 'state' and model to reflect moving nodes from temp to proper - note outarcs invalid after this point
        4) group nodes into connected groups
        5) determine which pipeline groups should be in by which pipelines the nodes were in
        
        6) calculate nodes that needed to be added/removed to get desired pipelines
        """
        #1
        try:
            state, new_nodes = self._pl.get()
        except RuntimeError:
            error("Couldn't communicate with pipeline, not updating")
            return
        if len(state.nodes) and not self.freeze:
            #2
            relabeling = self.pl_data.get_relabeling(new_nodes)
            #3
            #reassign the new nodes with optimised pipeline number from the temporary references originally given
            self._pl.reassign(relabeling)
            #renumber node_ids in state so that future references give right number
            #NOTE uses items not iteritems as this means its preevaluated, so can remove while looping
            for node_id, node in state.nodes.items():
                for input, inarc in node.inarcs.items():
                    if inarc[0] in relabeling:
                        node.inarcs[input] = (relabeling[inarc[0]], inarc[1])
                if node_id in relabeling:
                    state.nodes.pop(node_id)
                    state.nodes[relabeling[node_id]] = node
            #4
            traced = [] #nodes that have been added to a group
            groups = [] #node_list, pipelines
            info('Found %d new nodes.' %(len(new_nodes),))
            for node_id in state.nodes:
                if node_id in traced: continue #if the node is already part of a grouping, ignore it
                nodes = {} #node_id:node, list of nodes in this group
                pipelines = set()
                to_map = [node_id,] #nodes that need to be checked
                #map outwards as far as possible
                while len(to_map)>0:
                    for inarc in state.nodes[to_map[0]].inarcs.values():
                        #for all inputs, if we haven't already mapped it, or put it in the list to map
                        #then we need to 'map' it
                        if (not inarc[0] in traced) and (not inarc[0] in to_map):
                            to_map.append(inarc[0])
                    for outarcs in state.nodes[to_map[0]].outarcs.values():
                        for outarc in outarcs:
                            if (not outarc[0] in traced) and (not outarc[0] in to_map):
                                to_map.append(outarc[0])
                    #mark the node as traced, add to this group, add the pipeline its in to this group, remove it from the to check list
                    traced.append(to_map[0])
                    nodes[to_map[0]] = state.nodes[to_map[0]]
                    try:
                        pipelines.add(self.pl_data.node_id2pipeline[to_map[0]])
                    except KeyError:
                        #isn't in a pipeline
                        pass
                    to_map.remove(to_map[0])
                groups.append((nodes,pipelines))
            #5
            #deals with pipelines getting merged (by connecting)
            merged_pls = [] #[(pipelines,nodes)]
            pl2merged_pl = {} #pipeline, merged_pl index
            for group, pipelines in groups:
                for pipeline in pipelines:
                    if pipeline in pl2merged_pl:
                        merged_pl_id = pl2merged_pl
                        break
                else:
                    merged_pl_id = len(merged_pls)
                    merged_pls.append([set(),{}])
                for pipeline in pipelines:
                    pl2merged_pl[pipeline] = merged_pl_id
                merged_pls[merged_pl_id][1].update(group)
                merged_pls[merged_pl_id][0].update(pipelines)
            self.pl_data.merge_and_update(merged_pls)
        #7
        #set current state to state from pipeline
        self.pl_state = state
        #generate 'clean' new state
        new_state = pipeline.State()
        for reqname2reqs in self.requests.values():
            for reqs in reqname2reqs.values():
                for req in reqs:
                    new_state.nodes.update(self.pl_data.pipelines[self.pl_data.request2filename[req]].nodes)
        for req in self.detector_requests:
            if not req in self.pl_data.pipelines:
                error('Requested non-existant pipeline %s' %(req, ))
                continue
            new_state.nodes.update(self.pl_data.pipelines[self.pl_data.request2filename[req]].nodes)
        self.cur_det_reqs = self.detector_requests
        #8
        add_nodes = {}
        remove_nodes = {}
        #calculate nodes to remove
        for node_id, node in self.pl_state.nodes.items():
            if (not node_id in new_state.nodes):
                remove_nodes[node_id] = node
        #calculate nodes to add
        for node_id, node in new_state.nodes.items():
            if not node_id in self.pl_state.nodes:# and (not self.disable_gui or node.type != int(messaging.NodeType.GuiOutput)):
                add_nodes[node_id] = node
        #update pl_state
        self.pl_state.nodes.update(add_nodes)
        for node_id in remove_nodes:
            self.pl_state.nodes.pop(node_id)
        self._pl.add(add_nodes)
        self._pl.remove(remove_nodes)
    def run(self):
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
                self.eval_branches()
                self.reeval = False
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
    pm = pipelineManager('pipeline_manager')
    try:
        pm._setup(**opts.__dict__)
        pm.run()
    finally:
        pm.die()
