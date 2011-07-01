from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import pipeline

import threading, os.path, hashlib, cPickle, time, traceback, shelve
from glob import glob

"""
NOTE - if broken: did someone change the node type numbers, or edit the copy node, since copy nodes are manually added, so type needs to be updated etc
                  did outarcs get dropped/become important? this uses output arcs when importing pipelines, but ignores them afterwards (e.g. when setting up pipelines)
TODO - more advanced optimisation
     - actually save optimised pipelines... inc. ones to avoid?
     - do copy nodes copy other data types? something to watch out for...
     - strip of GUI outputs (and related processing)...
     - save state (only once pieline succefully loaded) and restore on restart
"""

#lets modify pipeline.Model class a bit so its a bit more flexible
class NewModel(pipeline.Model):
    def __init__(self, *args, **kwargs):
        pipeline.Model.__init__(self, *args, **kwargs)
        self.manager2pl = {}#node ids here: node ids in pl
        self.pl2manager = {}
        self.temp_number = 0
    def get(self, timeout=3.0): #needs nnid to know where new nodes are out of the way
        state = pipeline.Model.get(self, timeout)
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
        return renumbered_state, new_nodes
    def reassign(self, relabel):
        for old_id, new_id in relabel.items():
            self.pl2manager[self.manager2pl[old_id]] = new_id
            self.manager2pl[new_id] = self.manager2pl[old_id]
            self.manager2pl.pop(old_id)
    def set(self, state):
        raise NotImplementedError
    def add(self, nodes):
        for node_id, node in nodes.items():
            if node_id in self.manager2pl:
                warning('Node already added, skipping')
                continue
            try:
                new_node_id = self.addSynchronous(node.type)
            except RuntimeError:
                error("Couldn't add node %d, skipping" %(new_node_id))
                continue
            self.manager2pl[node_id] = new_node_id
            self.pl2manager[new_node_id] = node_id
            for param, value in node.params.items():
                try:
                    self.setParameterSynchronous(new_node_id, param, value)
                except RuntimeError:
                    error("Couldn't set parameter: "+str((param,value)))
        #now all added, add arcs
        for node_id, node in nodes.items():
            for input, (from_node_id, from_node_field) in node.inarcs.items():
                try:
                    self.addArcSynchronous(self.manager2pl[from_node_id], from_node_field, self.manager2pl[node_id], input)
                except RuntimeError:
                    error("Couldn't add arc from %d (%s) to %d (%s) (%d to %d in pipeline)" %(from_node_id, from_node_field, node_id, input, self.manager2pl[from_node_id], self.manager2pl[node_id]))
    def remove(self, nodes):
        for node_id in nodes:
            try:
                try:
                    self.removeSynchronous(self.manager2pl[node_id])
                except RuntimeError:
                    error("Couldn't remove node: pipeline did not respond")
                #want to remove nodes regardless otherwise wont be able to add them again
                self.pl2manager.pop(self.manager2pl[node_id])
                self.manager2pl.pop(node_id)
            except KeyError:
                error("Couldn't remove node: no record of node exists")

class Branch():
    def __init__(self, nodes):
        #inputs = {input_node_id: (branch_id, output_node_id),}
        self.nodes = nodes
    def compare(self, other_branch):
        #first check to see if same number of nodes
        if len(self.nodes)!=len(other_branch.nodes):
            return None
        #split into equivalent nodes
        node_map = {} #node_id:[other_node_ids]
        for node_id, node in self.nodes.items():
            node_map[node_id] = []
            for other_node_id, other_node in other_branch.nodes.items():
                if node.type == other_node.type and node.params == other_node.params:
                    node_map[node_id].append(other_node_id)
        #check there are the right no. of equivalent nodes, and relabel where possible
        relabel = {} #self_label: other_label
        possible = {} #list of nodes that aren't in relabel as there are more than one possibility, node_id: possible_node_ids
        for node_id, other_node_ids in node_map.items():
            if len(other_node_ids) == 0:
                #no possible matches, no relabeling
                return None
            elif len(other_node_ids) == 1:
                #check hasnt already been used
                if other_node_ids[0] in relabel.values():
                    return None
                #if not already used, if relabeling exist must relabel to this
                relabel[node_id] = other_node_ids[0]
            else:
                #multiple possible nodes match
                possible[node_id] = other_node_ids
        #now generate possible relabellings
        #basically known possibilities, and for a node that hasn't yet been relabeled, add all the possible things it could be relabeled to
        possibles = [relabel,] #list of possisible relabels (starts of with those that have just one possibility)
        for self_node_id, other_node_ids in possible.items():
            new_possibles = []
            for relabeling in possibles:
                for other_node_id in other_node_ids:
                    #if the other_node_id has already been used, don't create a new relabelling since that would relabel 2 things to the same thing
                    if not other_node_id in relabeling.values():
                        #very inefficient copying...
                        new_relabeling = dict([(x,y) for x,y in relabeling.items()])
                        new_relabeling[self_node_id] = other_node_id
                        new_possibles.append(new_relabeling)
            possibles = new_possibles
        #now check to see whether any relabelings give an equivalent branch
        for relabeling in possibles:
            if self.equiv(other_branch, relabeling):
                return relabeling
        #no possibilities gave a matching
        return None
    def equiv(self, other_branch, relabel):
        #basically, take advantage of pythons ability to compare dictionaries
        #compare dict of nodes, compring nodes would always fail as they are different instances
        other_branch_dict = dict([(node_id, node.__dict__) for node_id, node in other_branch.nodes.items()])
        #careful not to actually relabel, create new copy instead (otherwise we'll relabel the original, and everything will be broken)
        relabeled_dict = {}
        for node_id, node in self.nodes.items():
            copied_node_dict = {'type':node.type,'params':node.params,'inarcs':{},'outarcs':{}}
            for input, inarc in node.inarcs.items():
                copied_node_dict['inarcs'][input] = (relabel[inarc[0]], inarc[1])
            #ignore outarcs, not used anyway
            #for output, outarcs in node.outarcs.items():
            #    copied_node_dict['outarcs'][output] = [(relabel[outarc[0]], outarc[1]) for outarc in outarcs]
            relabeled_dict[relabel[node_id]] = copied_node_dict
        #check all the nodes exist in both (otherwise we'll get an error later)
        if set(relabeled_dict.keys()) != set(other_branch_dict.keys()):
            return False
        #more complicated since we are ignoring outarcs so cannot just compare __dict__
        return all([all([other_branch_dict[nodeid][prop] == relabeled_dict[nodeid][prop] for prop in ('type', 'inarcs', 'params')]) for nodeid in relabeled_dict])
    def split(self, node_groups):
        branches = {} #branch_id, branch
        branch_id = 1
        #check that node_groups doesn't overlap and covers all nodes
        covered_nodes = []
        for node_group in node_groups:
            for node_id in node_group:
                if node_id in covered_nodes: raise Exception('Tried to split a branch into overlapping node groups')
                else: covered_nodes.append(node_id)
        #add the remaining nodes not in a group to the last group
        uncovered = [node_id for node_id in self.nodes if not node_id in covered_nodes]
        if len(uncovered): node_groups.append(uncovered)
        for node_group in node_groups:
            node_group_nodes = {}
            #create new branch based on nodes
            for node_id in node_group:
                node_group_nodes[node_id] = self.nodes[node_id]
            branches[branch_id] = Branch(node_group_nodes)
            branch_id += 1
        return branches
    def renumber(self, relabel):
        #renumber inputs (e.g. if branches it happens to depend on are equivalent to existing branches, we want to renumber to depend on nodes in the already existing branches)
        for node in self.nodes.values():
            new_inarcs = {}
            for input, inarc in node.inarcs.items():
                new_inarcs[input] = (relabel[inarc[0]], inarc[1]) if inarc[0] in relabel else inarc
            node.inarcs = new_inarcs
            
class oPipeline():
    #basically represents the original pipelines
    def __init__(self, branches, md5):
        self.branches = branches
        self.md5 = md5
    
class OptimisedPipelines():
    def __init__(self):
        self.branches = {} #branch_id, branch
        self.nodeid2branch = {}
        self.pipelines = {}
        self.old_pipelines = {} #use to store, e.g. if branch is update
        self.names = []
        self.nbid = 0
        self.nnid = 0
    def add_branches(self, branches, prefix='temp'):
        #add branches that may already have dependancies. things with prefix are new nodes
        mapping = {} #old:new
        max_n = 0
        #relabel nodes
        for branch in branches.values():
            new_nodes = {}
            for node_id, node in branch.nodes.items():
                if node_id[:len(prefix)] == prefix:
                    new_nodes[self.nnid+max_n] = node
                    mapping[node_id] = self.nnid+max_n
                    max_n+1
            #relabel inarcs
            for node in branch.nodes.values():
                for input, inarc in node.inarcs.items():
                    if inarc[0] in mapping:
                        node.inarcs[input] = (mapping[inarc[0]], inarc[1])
        self.nnid += max_n
        branch_relabeling, node_relabeling = self.add(branches)
        node_relabeling2 = {}
        for old_node_id, med_node_id in mapping.items():
            node_relabeling2[old_node_id] = node_relabeling[med_node_id] if med_node_id in node_relabeling else med_node_id
        return branch_relabeling, node_relabeling2
    def add_pl(self, pl_name, branches, md5):
        #add branches that have no already existing dependancies
        #renumber nodes so that they are above the highest node id already in optimised pipelines (this way we don't have to renumber them later)
        #for simplicity sake, just add the nnid (next node id) to all the node ids
        max_n = self.nnid
        for branch in branches.values():
            new_nodes = {}
            for nodeid, node in branch.nodes.items():
                new_node = pipeline.Node(nodeid+self.nnid, node.type, node.params)
                for input, (innodeid, field) in node.inarcs.items():
                    new_node.inarcs[input] = (innodeid+self.nnid, field)
                for output, outarcs in node.outarcs.items():
                    new_node.outarcs[output] = [(outnodeid+self.nnid, field) for outnodeid, field in outarcs]
                new_nodes[nodeid+self.nnid] = new_node
                max_n = self.nnid+nodeid if self.nnid+nodeid>max_n else max_n
            branch.nodes = new_nodes
        self.nnid = max_n + 1
        branch_relabeling, node_relabeling = self.add(branches)
        #finally add the pipeline
        self.pipelines[pl_name] = oPipeline(branch_relabeling.values(), md5)
    def add(self, branches):
        #add branches that have already had their node numbering sorted to not conflict with existing branches
        branch_relabeling = {}
        node_relabeling = {}
        #check whther branch already exists, else add as new branch, and work out relabelling
        #note we cant compare branches unless all the thing they depend on have been relabeled (otherwise dependancies check will fail)
        branch_ids = branches.keys() #branches to process
        counter = 0
        no_change = 0
        while len(branch_ids) != 0:
            counter = counter%len(branch_ids)
            branch_id = branch_ids[counter]
            branch = branches[branch_id]
            #check to see whether all the branches it depends
            if not all([all([((inarc[0] in branch.nodes) or (inarc[0] in self.nodeid2branch)) for inarc in node.inarcs.values()]) for node in branch.nodes.values()]):
                #not ready yet, skip for the moment
                counter = counter+1
                #check to stop us infinite looping
                if no_change > len(branch_ids):
                    error('Could not add branch of image pipeline, either circular links or depends on non-existant nodes')
                    break
                no_change += 1
                continue
            no_change = 0
            for branch2_id, branch2 in self.branches.items():
                relabel = branch.compare(branch2) #returns relabelling branch => branch2
                #existing branch
                if relabel:
                    #need to relabel other branches not yet added accordingly
                    for branch3 in branches.values():
                        branch3.renumber(relabel)
                    branch_relabeling[branch_id] = branch2_id
                    node_relabeling.update(relabel)
                    break
            else:
                #there were no relabelings for any existing branch, so this is not the same as any existing branch, so add it
                self.branches[self.nbid] = branch
                branch_relabeling[branch_id] = self.nbid
                #add all the nodes to nodeid2branch
                #check to see if names already used (print warning if so)
                for node_id, node in branch.nodes.items():
                    self.nodeid2branch[node_id] = self.nbid
                    if 'name' in node.params:
                        if node.params['name'] in self.names:
                            warning('Node name %s used more than once' %(node.params['name']))
                        else:
                            self.names.append(node.params['name'])
                self.nbid += 1
            branch_ids.remove(branch_id)
        return branch_relabeling, node_relabeling

class pipelineManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'pipeline_manager')
        self.request_lock = threading.RLock()
        self.setup_requests = []
        self.drop_requests = []
        self.requests = {'script':{}, 'other':{}} #detector_name: pl_names
        self.det_reqs = []
        self.cur_det_reqs = []
        self.pipeline_lock = threading.RLock()
        with self.pipeline_lock:
            #this may take time, so lock since other threads may start requesting as soon as they start
            self.load_pl_data()
            #self.script_pl = pipeline.Model(self.node, 'default')
            self._pl = NewModel(self.node, 'default')
            #save any old data just in case
            self._pl.save('%d_pl.temp' %(time.time(),))
            self._pl.clear()
            self.pl_state = pipeline.State()
            self.node_mapping = {}#node_ids here: node_ids in pipeline
    def load_pl_data(self):
        with self.pipeline_lock:
            #try and load optimised pipelines
            self.shelf = shelve.open('optimised.pipes')
            try:
                self.pl_data = self.shelf['pl_data']
                info('Found optimised pipelines data')
            except KeyError:
                self.pl_data = OptimisedPipelines()
            #scan the pipelines
            pipeline_names = [x[10:] for x in glob(os.path.join('pipelines', '*.pipe'))] #we'll drop the 'pipelines/' prefix
            for pl_name in pipeline_names:
                try:
                    info('Loading pipeline %s' %(pl_name,))
                    #for all new pipelines
                    if not pl_name in self.pl_data.pipelines:
                        #add new pl
                        self.add_pl(pl_name)
                        warning('Found a new pipeline, %s, has been auto added to optimised pipelines file' %(pl_name,))
                    #for all modified pipelines
                    elif hashlib.md5(open(os.path.join('pipelines', pl_name)).read()).hexdigest() != self.pl_data.pipelines[pl_name].md5:
                        #move old version to old_pipelines
                        self.pl_data.old_pipelines[pl_name+str(time.time())] = self.pl_data.pipelines.pop(pl_name)
                        #add new version
                        self.add_pl(pl_name)
                        warning('Pipeline %s was externally modified. The old version has been moved to old_pipelines.' %(pl_name,))
                except Exception as e:
                    error('Could not load pipeline %s' %(pl_name,))
                    traceback.print_exc()
    def add_pl(self, pl_name):
        with self.pipeline_lock:
            #load pipeline state
            pl_state = cPickle.load(open(os.path.join('pipelines', pl_name)))
            md5result = hashlib.md5(open(os.path.join('pipelines', pl_name)).read()).hexdigest()
            #find out which nodes are 'inputs', and what the next node id should be
            inputs = []
            next_id = 1
            for node_id, node in pl_state.nodes.items():
                if all([x[0]==0 for x in node.inarcs.values()]): inputs.append(node_id)
                if next_id <= node_id: next_id = node_id + 1
                #get rid of troublesome zeros
                for input, inarc in node.inarcs.items():
                    if inarc[0] == 0:
                        node.inarcs.pop(input)
            input_node2branch = {}
            for node_id in inputs:
                node = pl_state.nodes[node_id]
                #add copy to avoid interference between branches
                #may need multiple copies if input has multiple outputs
                #check to see whether anything actually uses it
                if not len(node.outarcs):
                    continue
                for output_name, outarcs in node.outarcs.items():
                    #if copy node changes, need to change this bit
                    copy_node = pipeline.Node(next_id, 1, inputarcs={'image': (node_id, output_name)}, outputarcs={'image copy': outarcs})
                    for to_node_id, to_node_field in outarcs:
                        pl_state.nodes[to_node_id].inarcs[to_node_field] = (next_id, 'image copy')
                    node.outarcs[output_name] = [(next_id, 'image')]
                    pl_state.nodes[next_id] = copy_node
                    next_id += 1
            #create all encompassing branch, then split off inputs (so camera etc do not get added to the pipeline tomany times)
            branches = Branch(pl_state.nodes).split([(x,) for x in inputs])
            self.pl_data.add_pl(pl_name, branches, md5result)
    @external_function
    def save_mods(self, pipelines=None, temporary=False):
        #if we change the pipeline, we need to be able to save changes
        pass
    @external_function
    def clear_old(self):
        #get rid of old data (e.g.pipelines that have been deleted
        pass
    @external_function
    def request_pl(self, requestor_type, requestor_name, requested_pl):
        #VERY IMPORTANT: make sure requestor_name is same as process_name
        #since the python thread cannot be interrupted multiple times it seems, and needs to be interrupted to set up the pipeline, request must be handled in the main process
        with self.request_lock:
            self.setup_requests.append({'requestor_type':requestor_type, 'requestor_name':requestor_name, 'requested_pl':requested_pl})
    @external_function
    def drop_pl(self, requestor_type, requestor_name, requested_pl):
        with self.request_lock:
            self.drop_requests.append({'requestor_type':requestor_type, 'requestor_name':requestor_name, 'requested_pl':requested_pl})
    @external_function
    def drop_all_pl(self, requestor_type, requestor_name):
        with self.request_lock:
            for pl_name in self.requests[requestor_type][requestor_name]:
                self.drop_pl(requestor_type, requestor_name, pl_name)
    @external_function
    def set_detector_pl(self, req_list):
        with self.request_lock:
            self.det_reqs = req_list
    def setup_pl(self, requestor_type, requestor_name, requested_pl):
        if requestor_type == 'script':
            #clear out other script requests on the basis that a) script won't request close together or b) the running script will be last to request
            with self.request_lock:
                for script_name in self.requests['script']:
                    if script_name != requestor_name:
                        for pl_name in self.requests['script'][script_name]:
                            self.drop_pl(requestor_type, requestor_name, pl_name)
        elif requestor_type == 'other':
            pass
        else:
            error('Invalid requestor type %s, named %s' %(requestor_type, requestor_name))
            return
        if not requested_pl in self.pl_data.pipelines:
            error('Non-existant pipeline requested')
            return
        with self.request_lock:
            #update request info
            if requestor_name in self.requests[requestor_type]:
                self.requests[requestor_type][requestor_name].append(requested_pl)
            else:
                self.requests[requestor_type][requestor_name] = [requested_pl]
        #confirm setup (atm only scripts)
        if requestor_type == 'script':
            getattr(self.ai, requestor_name).pl_response()
    def unsetup_pl(self, requestor_type, requestor_name, requested_pl):
        with self.request_lock:
            if not requestor_type in self.requests:
                error('Invalid type in requested drop pl')
            elif not requestor_name in self.requests[requestor_type]:
                error('Requestor does not have any pipelines to drop')
            elif not requested_pl in self.requests[requestor_type][requestor_name]:
                error('Requestor has not requested this pipeline %s' %(requested_pl,))
            else:
                if not requested_pl in self.pl_data.pipelines:
                    warning('Could not drop request %s, may have left things in the pipeline.' %(pl_name, ))
                else:
                    self.requests[requestor_type][requestor_name].remove(requested_pl)
    def eval_branches(self):
        with self.pipeline_lock:
            """
            1) get the current state (also returns a list of new nodes)
            2) group new nodes into branches
            2a) scan node for inputs and split off
            2b) TODO add copy node
            3) modify 'state' and model to reflect moving nodes from temp to proper - note outarcs invalid after this point
            4) scan existing nodes for:
                lost nodes - remove all traces
                changes to params
                changes to inarcs
            5) check pipeline dependencies
            6) if any of our new branches haven't been used, guess which pipeline they're supposed to be added to
            """
            #1
            state, new_nodes = self._pl.get()
            #2
            traced = []
            node_relabelings = {}
            new_branch_ids = []
            info('Found %d new nodes.' %(len(new_nodes),))
            for node_id in new_nodes:
                if node_id in traced: continue
                depends_on = []
                dependents = []
                nodes = {}
                to_map = [node_id,]
                #map outwards until you hit already existing things
                while len(to_map)>0:
                    for inarc in state.nodes[to_map[0]].inarcs.values():
                        if inarc[0] in new_nodes and (not inarc[0] in traced):
                            to_map.append(inarc[0])
                        else:
                            depends_on.append(to_map[0])
                    for outarcs in state.nodes[to_map[0]].outarcs.values():
                        for outarc in outarcs:
                            if outarc[0] in new_nodes and (not outarc[0] in traced):
                                to_map.append(outarc[0])
                            else:
                                dependents.append(to_map[0])
                    traced.append(to_map[0])
                    nodes[to_map[0]] = state.nodes[to_map[0]]
                    to_map.remove(to_map[0])
                #check for inputs
                inputs = []
                for node_id, node in nodes.items():
                    if len(node.inarcs) == 0:
                        inputs.append(node_id)
                #TODO add copy node
                #create new branches
                branches = Branch(nodes).split([[x] for x in inputs])
                branch_relabeling, node_relabeling = self.pl_data.add_branches(branches)
                node_relabelings.update(node_relabeling)
                new_branch_ids.extend(branch_relabeling.values())
            #3
            self._pl.reassign(node_relabelings)
            for node_id, node in state.nodes.items():
                for input, inarc in node.inarcs.items():
                    if inarc[0] in node_relabelings:
                        node.inarcs[input] = (node_relabelings[inarc[0]], inarc[1])
                if node_id in node_relabelings:
                    state.nodes.pop(node_id)
                    state.nodes[node_relabelings[node_id]] = node
            #4
            for node_id, node in self.pl_state.nodes.items():
                #check for nodes lost
                if not node_id in state.nodes:
                    info('Detected node %d removed.' %(node_id,))
                    #node lost, remove all traces
                    for branch in self.pl_data.branches.values():
                        for node_id2, node2 in branch.nodes.items():
                            if node_id2 == node_id:
                                branch.nodes.pop(node_id)
                                continue
                            for input, inarc in node2.inarcs.items():
                                if inarc[0] == node_id:
                                    node2.inarcs.pop(input)
                    continue
                #check for properties changed on existing nodes
                if node.params != state.nodes[node_id].params:
                    info('Detected node %d params changed.' %(node_id,))
                    self.pl_data.branches[self.pl_data.nodeid2branch[node_id]].nodes[node_id].params = state.nodes[node_id].params
                #and check for changed inarcs on existing nodes
                for input, inarc in node.inarcs.items():
                    if input in state.nodes[node_id].inarcs:
                        if state.nodes[node_id].inarcs[input] != inarc:
                            #inarc changed
                            info('Detected node %d input to %s changed.' %(node_id, input))
                            self.pl_data.branches[self.pl_data.nodeid2branch[node_id]].nodes[node_id].inarcs[input] = state.nodes[node_id].inarcs[input]
                        #no change
                    else:
                        #inarc removed
                        info('Detected node %d input to %s removed.' %(node_id, input))
                        self.pl_data.branches[self.pl_data.nodeid2branch[node_id]].nodes[node_id].inarcs.pop(input)
                for input, inarc in state.nodes[node_id].inarcs.items():
                    if not input in node.inarcs:
                        #new inarc
                        info('Detected node %d new input to %s.' %(node_id, input))
                        self.pl_data.branches[self.pl_data.nodeid2branch[node_id]].nodes[node_id].inarcs[input] = (node_relabelings[inarc[0]],inarc[1]) if state.nodes[node_id].inarcs[input][0] in node_relabelings else inarc
            #5
            #check to see whether any pipeline branch dependancies have changed
            for pl_name, pl in self.pl_data.pipelines.items():
                #check to see if any new branches haven't been added somewhere
                #find out which branches this is dependant on
                to_check = list(pl.branches)
                depends_on_branches = []
                while len(to_check):
                    for node in self.pl_data.branches[to_check[0]].nodes.values():
                        for node2_id, node2field in node.inarcs.values():
                            if (not self.pl_data.nodeid2branch[node2_id] in depends_on_branches):
                                to_check.append(self.pl_data.nodeid2branch[node2_id])
                                depends_on_branches.append(self.pl_data.nodeid2branch[node2_id])
                    if to_check[0] in new_branch_ids:
                        new_branch_ids.remove(to_check[0])
                    to_check.remove(to_check[0])
                #add branches to pl
                info('Adding '+str(depends_on_branches)+' to '+pl_name+'.')
                for branch in depends_on_branches:
                    if not branch in pl.branches:
                        pl.branches.append(branch)
            #6
            #if for nodes which things aren't dependant on
            if len(new_branch_ids): info('Found %d new branches that no pipeline depends on, guessing...' %(len(new_branch_ids),))
            for branch_id in new_branch_ids:
                #if its attached to just one pl, then add to that pl
                to_check = [branch_id]
                depends_on_branches = []
                while len(to_check):
                    for node in self.pl_data.branches[to_check[0]].nodes.values():
                        for node2_id, node2field in node.inarcs.values():
                            if (not self.pl_data.nodeid2branch[node2_id] in depends_on_branches) and (not self.pl_data.nodeid2branch[node2_id] == branch_id):
                                to_check.append(self.pl_data.nodeid2branch[node2_id])
                                depends_on_branches.append(self.pl_data.nodeid2branch[node2_id])
                    to_check.remove(to_check[0])
                info('New branch dependant on '+str(depends_on_branches))
                attached_to = {}
                for pl_name, pl in self.pl_data.pipelines.items():
                    for branch_id in depends_on_branches:
                        if branch_id in pl.branches:
                            continue
                        break
                    else:
                        #this is attached to this pl
                        attached_to[pl_name] = pl
                        info('New branch could be attached to %s' %(pl_name, ))
                if len(attached_to) == 1:
                    info('Adding to pipeline %s.' %(attached_to.keys()[0],))
                    attached_to[attached_to.keys()[0]].branches.extend(branch_relabeling.values())
                #TODO create pl. possibly add request
            #save modifications
            self.shelf['pl_data'] = self.pl_data
            #set current state to state from pipeline
            self.pl_state = state
            #generate 'clean' new state
            new_state = pipeline.State()
            for reqname2reqs in self.requests.values():
                for reqs in reqname2reqs.values():
                    for req in reqs:
                        for branch_id in self.pl_data.pipelines[req].branches:
                            new_state.nodes.update(self.pl_data.branches[branch_id].nodes)
            for req in self.det_reqs:
                if not req in self.cur_det_reqs:
                    changed = True
                    if not req in self.pl_data.pipelines:
                        error('Requested non-existant pipeline %s' %(req, ))
                        continue
                    for branch_id in self.pl_data.pipelines[req].branches:
                        new_state.nodes.update(self.pl_data.branches[branch_id].nodes)
            self.cur_det_reqs = self.det_reqs
            add_nodes = {}
            remove_nodes = {}
            #calculate nodes to remove
            for node_id, node in self.pl_state.nodes.items():
                if (not node_id in new_state.nodes):
                    remove_nodes[node_id] = node
            #calculate nodes to add
            for node_id, node in new_state.nodes.items():
                if not node_id in self.pl_state.nodes:
                    add_nodes[node_id] = node
            #update pl_state
            self.pl_state.nodes.update(add_nodes)
            for node_id in remove_nodes:
                self.pl_state.nodes.pop(node_id)
            self._pl.add(add_nodes)
            self._pl.remove(remove_nodes)
    def run(self):
        while True:
            with self.request_lock:
                changed = len(self.setup_requests)+len(self.drop_requests)
                for setup_request in self.setup_requests:
                    try:
                        self.setup_pl(**setup_request)
                    except Exception:
                        error('Exception while trying to setup pipeline caused by '+str(setup_request))
                        traceback.print_exc()
                self.setup_requests = []
                for drop_request in self.drop_requests:
                    try:
                        self.unsetup_pl(**drop_request)
                    except Exception:
                        error('Exception while trying to drop pipeline request caused by '+str(drop_request))
                        traceback.print_exc()
                self.drop_requests = []
                if set(self.det_reqs) != set(self.cur_det_reqs):
                    changed = True
            #reevaluate if changed
            if changed:
                self.eval_branches()
                self.ai.detector_control.update_pl_requests(self.cur_det_reqs)
            time.sleep(1)
        
if __name__ == '__main__':
    pm = pipelineManager()
    pm.run()