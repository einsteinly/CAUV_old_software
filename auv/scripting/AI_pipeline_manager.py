from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import pipeline

import threading, os.path, md5, cPickle, time, traceback
from glob import glob

"""
NOTE - if broken: did someone change the node type numbers, or edit the copy node, since copy nodes are manually added, so type needs to be updated etc
                  did outarcs get dropped/become important? this uses output arcs when importing pipelines, but ignores them afterwards (e.g. when setting up pipelines)
TODO - more advanced optimisation
     - actually save optimised pipelines... inc. ones to avoid?
     - do copy nodes copy other data types? something to watch out for...
     - strip of GUI outputs (and related processing)...
     - save state (only once pieline succefully loaded) and restore on restart
     - make branch.split actually work out branch.depends properly
"""

class Branch():
    def __init__(self, nodes, depends):
        #inputs = {input_node_id: (branch_id, output_node_id),}
        self.nodes = nodes
        self.depends = depends
    def compare(self, other_branch):
        #first check to see if same number of nodes
        if len(self.nodes)!=len(other_branch.nodes):
            return None
        #check has same dependencies
        if set(self.depends) != set(other_branch.depends):
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
                for self_node_id in self_node_ids:
                    possible[self_node_id] = other_node_ids
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
        node2branch = {}
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
                node2branch[node_id] = branch_id
            branches[branch_id] = Branch(node_group_nodes, [])
            branch_id += 1
        #for nodes that have arc to other branches, we need to set depends accordingly
        for branch in branches.values():
            for node_id, node in branch.nodes.items():
                for input_name, arc in node.inarcs.items():
                    #other_node_id = arc[0]
                    #if the other node is in the current branch, we don't need to do anything
                    #watch out for 0 case, this means no input
                    if arc[0] in branch.nodes: pass
                    #try look it up ina nother branch
                    elif arc[0] in node2branch:
                        if not node2branch[arc[0]] in branch.depends:
                            branch.depends.append(node2branch[arc[0]]) #note node_id must be arc[0] since we have not changed node ids from original branch
                    #if the other node is not in the current branch, nor the whole branch take it from inputs
                    else:
                        #TODO make this actually look up branches, otherwise to many dependancies
                        for branchid in self.depends:
                            if not branchid in branch.depends:
                                branch.depends.append(branchid)
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
        self.pipelines = {}
        self.old_pipelines = {} #use to store, e.g. if branch is update
        self.names = []
        self.nbid = 0
        self.nnid = 0
    def add_pl(self, pl_name, branches, md5):
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
        #note, all dependant branches should be added simultaneously as they will be relabelled to avoid conflicts
        included_branches = []
        #check whther branch already exists, else add as new branch, and work out relabelling
        #note we cant compare branches unless all the thing they depend on have been relabeled (otherwise dependancies check will fail)
        branch_ids = branches.keys() #branches to process
        counter = -1
        while len(branch_ids) > 0:
            counter = (counter+1)%len(branch_ids)
            branch_id = branch_ids[counter]
            branch = branches[branch_id]
            #check to see whether all the branches it depends
            if not all([(in_branch in self.branches) for in_branch in branch.depends]):
                #not ready yet, skip for the moment
                #we'd better hope there aren't any circular links...
                continue
            for branch2_id, branch2 in self.branches.items():
                relabel = branches[branch_ids[counter]].compare(branch2) #returns relabelling branch => branch2
                #existing branch
                if relabel:
                    #need to relabel depends other branches not yet added accordingly
                    for branch3 in branches.values():
                        for dep_branch_id in branch3.depends:
                            if dep_branch_id == branch_id:
                                branch3.depends.remove(branch_id)
                                branch3.depends.append(branch2_id)
                        #relabel any nodes referring to the old numbers
                        branch3.renumber(relabel)
                    included_branches.append(branch2_id)
                    break
            else:
                #there were no relabelings for any existing branch, so this is not the same as any existing branch, so add it
                #only need to relabel branch id in the remaining unadded branches
                for branch3 in branches.values():
                    for out_branch_id in branch3.depends:
                        if out_branch_id == branch_id:
                            branch3.depends.remove(branch_id)
                            branch3.depends.append(self.nbid)
                included_branches.append(self.nbid)
                self.branches[self.nbid] = branch
                #check to see if names already used (print warning if so)
                for node_id, node in branch.nodes.items():
                    if 'name' in node.params:
                        if node.params['name'] in self.names:
                            warning('Node name %s used more than once' %(node.params['name']))
                        else:
                            self.names.append(node.params['name'])
                self.nbid += 1
            branch_ids.remove(branch_id)
        #finally add the pipeline
        self.pipelines[pl_name] = oPipeline(included_branches, md5)

class pipelineManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'pipeline_manager')
        self.request_lock = threading.RLock()
        self.setup_requests = []
        self.drop_requests = []
        self.requests = {'script':{}, 'detector':{}} #detector_name: pl_names
        self.pipeline_lock = threading.RLock()
        with self.pipeline_lock:
            #this may take time, so lock since other threads may start requesting as soon as they start
            self.load_pl_data()
            #self.script_pl = pipeline.Model(self.node, 'default')
            self.detector_pl = pipeline.Model(self.node, 'default')
            self.branches = {} #branch_id: no of time branch in use
            #save any old data just in case
            self.detector_pl.save('%d_detector_pl.temp' %(time.time(),))
            self.detector_pl.clear()
    def load_pl_data(self):
        with self.pipeline_lock:
            #try and load optimised pipelines
            try:
                self.pl_data = cPickle.load(open(os.path.join('pipelines', 'optimised.opipe'))) #must not end in pipe or will try and import self
            except IOError:
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
                    elif md5.md5(open(os.path.join('pipelines', pl_name)).read()) != self.pl_data.pipelines[pl_name].md5:
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
            md5result = md5.md5(open(os.path.join('pipelines', pl_name)).read())
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
            branches = Branch(pl_state.nodes, depends=[]).split([(x,) for x in inputs])
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
            for req_id in self.name2request[requestor_type][requestor_name]:
                self.drop_pl(req_id)
            self.name2request[requestor_type].pop(requestor_name)
    def setup_pl(self, requestor_type, requestor_name, requested_pl):
        if requestor_type == 'script':
            #clear out other script requests on the basis that a) script won't request close together or b) the running script will be last to request
            with self.request_lock:
                for script_name in self.requests['script']:
                    if script_name != requestor_name:
                        for pl_name in self.requests['script'][script_name]:
                            self.drop_pl(requestor_type, requestor_name, pl_name)
        elif requestor_type == 'detector':
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
        for branch_id in self.pl_data.pipelines[requested_pl].branches:
            if branch_id in self.branches:
                self.branches[branch_id] += 1
            else:
                self.branches[branch_id] = 1
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
                    for branch_id in self.pl_data.pipelines[requested_pl].branches:
                        self.branches[branch_id] -= 1
    def eval_branches(self):
        with self.pipeline_lock:
            new_state = pipeline.State()
            for branch_id, no_req in self.branches.items():
                if no_req:
                    new_state.nodes.update(self.pl_data.branches[branch_id].nodes)
            self.detector_pl.set(new_state)
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
            #reevaluate if changed
            if changed: self.eval_branches()
            time.sleep(1)
        
if __name__ == '__main__':
    pm = pipelineManager()
    pm.run()