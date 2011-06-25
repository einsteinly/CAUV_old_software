from AI_classes import aiProcess, external_function

from cauv.debug import info, warning, error, debug
from cauv import pipeline

import threading, os.path, md5, cPickle, time, traceback
from glob import glob

"""
TODO - more advanced optimisation
-save state (only once pieline succefully loaded) and restore on restart.
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
        node_map = {} #ref_node: [node_ids],[other_node_ids]
        for node_id, node in self.nodes.items():
            #check to see whether equivalent nodes have already been added
            for ref_node in node_map:
                if ref_node.type == node.type and ref_node.params == node.params:
                    node_map[ref_node][0].append(node_id)
                    break
            #else add as new
            else:
                equiv_node_ids = []
                for other_node_id, other_node in other_branch.nodes.items():
                    if node.type == other_node.type and node.params == other_node.params:
                        equiv_node_ids.append(other_node_id)
                #if we didnt find any equivalents, there isnt a mapping
                if not len(equiv_node_ids):
                    return None
                node_map[node] = ([node_id,],equiv_node_ids)
        #check there are the right no. of equivalent nodes, and relabel where possible
        relabel = {} #self_label: other_label
        possible = {}
        for self_node_ids, other_node_ids in node_map.values():
            if len(self_node_ids)!=len(other_node_ids):
                return None
            elif len(self_node_ids) == 1:
                relabel[self_node_ids[0]]=other_node_ids[0]
            else:
                for self_node_id in self_node_ids:
                    possible[self_node_id] = other_node_ids
        #now generate possible relabellings
        possibles = [relabel,] #list of possisible relabels
        for self_node_id, other_node_ids in possible.items():
            new_possibles = []
            for relabeling in possibles:
                for other_node_id in other_node_ids:
                    #if the other_node_id has already been used, don't create a new relabelling
                    if not other_node_id in relabeling.values():
                        #very inefficient copying...
                        new_relabeling = dict([(x,y) for x,y in relabeling.items()])
                        new_relabeling[self_node_id] = other_node_id
                        new_possibles.append(new_relabeling)
            possibles = new_possibles
        for relabeling in possibles:
            if self.equiv(other_branch, relabeling):
                print 'Found 2 matching branches: ', self.__dict__, other_branch.__dict__
                return relabeling
        #no possibilities gave a matching
        return None
    def equiv(self, other_branch, relabel):
        #compare dict of nodes, compring nodes would always fail as they are different instances
        other_branch_dict = dict([(node_id, node.__dict__) for node_id, node in other_branch.nodes.items()])
        #careful not to actually relabel, create new copy instead
        relabeled_dict = {}
        for node_id, node in self.nodes.items():
            copied_node_dict = {'type':node.type,'params':node.params,'inarcs':{},'outarcs':{}}
            for input, inarc in node.inarcs.items():
                copied_node_dict['inarcs'][input] = (relabel[inarc[0]], inarc[1])
            #ignore outarcs, not used anyway
            #for output, outarcs in node.outarcs.items():
            #    copied_node_dict['outarcs'][output] = [(relabel[outarc[0]], outarc[1]) for outarc in outarcs]
            relabeled_dict[relabel[node_id]] = copied_node_dict
        if set(relabeled_dict.keys()) != set(other_branch_dict.keys()):
            return False
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
        for node in self.nodes.values():
            new_inarcs = {}
            for input, inarc in node.inarcs.items():
                new_inarcs[input] = (relabel[inarc[0]], inarc[1]) if inarc[0] in relabel else inarc
            node.inarcs = new_inarcs
class oPipeline():
    def __init__(self, branches, md5):
        self.branches = branches
        self.md5 = md5
    
class OptimisedPipelines():
    def __init__(self):
        self.branches = {} #branch_id, branch
        self.pipelines = {}
        self.old_pipelines = {}
        self.relabel = {}
        self.nbid = 0
        self.nnid = 0
        self.autoname = 0
    def add_pl(self, pl_name, branches, md5):
        #renumber nodes
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
        #note we cant compare branches unless all the thing they dpend on have been relabeled
        branch_ids = branches.keys() #branches to process
        counter = -1
        while len(branch_ids) > 0:
            counter = (counter+1)%len(branch_ids)
            branch_id = branch_ids[counter]
            branch = branches[branch_id]
            #check to see whether all the branches it depends
            if not all([(in_branch in self.branches) for in_branch in branch.depends]):
                #not ready yet, skip for the moment
                continue
            for branch2_id, branch2 in self.branches.items():
                relabel = branches[branch_ids[counter]].compare(branch2) #returns relabelling branch => branch2
                #existing branch
                if relabel:
                    #need to relabel accordingly
                    for branch3 in branches.values():
                        #update branches dependant on this branch
                        for dep_branch_id in branch3.depends:
                            if dep_branch_id == branch_id:
                                branch3.depends.remove(branch_id)
                                branch3.depends.append(branch2_id)
                        #relabel any nodes referring to the old numbers
                        branch3.renumber(relabel)
                    included_branches.append(branch2_id)
                    break
            else:
                #new branch
                #only need to relabel branch id
                for branch3 in branches.values():
                    for out_branch_id in branch3.depends:
                        if out_branch_id == branch_id:
                            branch3.depends.remove(branch_id)
                            branch3.depends.append(self.nbid)
                included_branches.append(self.nbid)
                self.branches[self.nbid] = branch
                #sort out renaming to avoid name clashes
                self.relabel[self.nbid] = {}
                for node_id, node in branch.nodes.items():
                    if 'name' in node.params:
                        self.relabel[self.nbid][node.params['name']] = str(self.autoname)
                        node.params['name'] = str(self.autoname)
                        self.autoname += 1
                self.nbid += 1
            branch_ids.remove(branch_id)
        #finally add the pipeline
        self.pipelines[pl_name] = oPipeline(included_branches, md5)

class pipelineManager(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'pipeline_manager')
        self.request_lock = threading.Lock()
        self.setup_requests = []
        self.pipeline_lock = threading.RLock()
        with self.pipeline_lock:
            #this may take time, so lock since other threads may start requesting as soon as they start
            self.load_pl_data()
            #self.script_pl = pipeline.Model(self.node, 'default')
            self.detector_pl = pipeline.Model(self.node, 'default')
            self.branches = {}
            self.requests = {}
            self.autoreq = 0
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
            pipeline_names = [x[10:] for x in glob(os.path.join('pipelines', '*.pipe'))]
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
            inputs = []
            outputs = []
            next_id = 1
            for node_id, node in pl_state.nodes.items():
                if all([x[0]==0 for x in node.inarcs.values()]): inputs.append(node_id)
                if all([not len(x) for x in node.outarcs.values()]): outputs.append(node_id)
                if next_id <= node_id: next_id = node_id + 1
                #get rid of troublesome zeros
                for input, inarc in node.inarcs.items():
                    if inarc[0] == 0:
                        node.inarcs.pop(input)
            input_node2branch = {}
            for node_id in inputs:
                node = pl_state.nodes[node_id]
                #split off inputs, add copy to avoid interference between branches
                #may need multiple copies if input has multiple outputs
                for output_name, outarcs in node.outarcs.items():
                    copy_node = pipeline.Node(next_id, 1, inputarcs={'image': (node_id, output_name)}, outputarcs={'image copy': outarcs})
                    for to_node_id, to_node_field in outarcs:
                        pl_state.nodes[to_node_id].inarcs[to_node_field] = (next_id, 'image copy')
                    node.outarcs[output_name] = [(next_id, 'image')]
                    pl_state.nodes[next_id] = copy_node
                    next_id += 1
            branches = Branch(pl_state.nodes, depends=[]).split([(x,) for x in inputs])
            self.pl_data.add_pl(pl_name, branches, md5result)
    @external_function
    def save_mods(self, pipelines=None, temporary=False):
        pass
    @external_function
    def clear_old(self):
        #get rid of old data (e.g.pipelines that have been deleted
        pass
    @external_function
    def request_pl(self, requestor_type, requestor_name, requested_pl, req_nodes, req_all=False):
        #since the python thread cannot be interrupted multiple times it seems, and needs to be interrupted to set up the pipeline, request must be handled in the main process
        with self.request_lock:
            self.setup_requests.append({'requestor_type':requestor_type, 'requestor_name':requestor_name, 'requested_pl':requested_pl, 'req_nodes':req_nodes, 'req_all':req_all})
    def setup_pl(self, requestor_type, requestor_name, requested_pl, req_nodes, req_all=False):
            required_branches = []
            node_names = {} #requested: new
            #search pipeline for node with req_node name
            for branch_id in self.pl_data.pipelines[requested_pl].branches:
                for node_id, node in self.pl_data.branches[branch_id].nodes.items():
                    if 'name' in node.params:
                        if node.params['name'] in req_nodes:
                            node_names[node.params['name']] = self.pl_data.rename[branch_id][node.params['name']]
                            if not branch_id in required_branches:
                                required_branches.append(branch_id)
            #unless req_all, fill in only needed branches
            if req_all:
                required_branches = self.pl_data.pipelines[requested_pl].branches
            request_id = str(self.autoreq)
            self.autoreq += 1
            self.requests[request_id] = required_branches
            for branch_id in required_branches:
                if branch_id in self.branches:
                    self.branches[branch_id] += 1
                else:
                    self.branches[branch_id] = 1
            #reevaluate
            self.eval_branches()
            #return new names
            getattr(self.ai, requestor_name).pl_response(request_id, node_names)
    def eval_branches(self):
        with self.pipeline_lock:
            new_state = pipeline.State()
            for branch_id, no_req in self.branches.items():
                if no_req:
                    new_state.nodes.update(self.pl_data.branches[branch_id].nodes)
            print new_state
            self.detector_pl.set(new_state)
    @external_function
    def drop_pl(self, request_id):
        with self.pipeline_lock:
            pass
    def run(self):
        while True:
            with self.request_lock:
                for setup_request in self.setup_requests:
                    self.setup_pl(**setup_request)
                self.setup_requests = []
            time.sleep(1)
        
if __name__ == '__main__':
    #Note pipelineManager is improted into task_manager to run, if started directly assume wnot runnin g as part of AI framework
    pm = pipelineManager()
    pm.run()