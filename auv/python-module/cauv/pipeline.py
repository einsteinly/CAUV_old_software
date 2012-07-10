'''This module provides an interface for manipulating the image processing pipeline.'''

import messaging
from debug import debug, warning, error, info

import threading
import pickle
import copy

#pylint: disable=E1101

def fromNPV(npv):
    return npv.value

def toNPV(value):
    return messaging.ParamValue.create(value);
 
class Node:
    def __init__(self, id, type, parameters = None, inputarcs = None, outputarcs = None):
        self.type = type
        self.params = parameters  # param : value
        self.inarcs = inputarcs   # input : (node id, output)
        self.outarcs = outputarcs # output : (node id, input)
        if self.params is None:
            self.params = {}
        if self.inarcs is None:
            self.inarcs = {}
        if self.outarcs is None:
            self.outarcs = {}
    def __repr__(self): 
        return "Node{%s, %s, %s, %s}" % (self.type, self.params, self.inarcs, self.outarcs)

class State:
    def __init__(self):
        self.nodes = {} # id : Node
    def __repr__(self):
        return str(self.nodes)


#
# BEWARE:
#  This little bit of unpickle machinery allows old saved pipelines to be
#  loaded if any of the data structures or messages involved in setting the
#  pipeline state change.
#
#  You SHOULD NOT RELY ON THESE HOOKS on anything other than a temporary basis:
#  once you can load your old pipeline, save it in the new format, check that
#  it works, then remove all copies of the old pipeline, and remove the hook
#  code that you add here.
#

def makeNewLocalNodeInput(input, subtype, schedType=messaging.InputSchedType.Must_Be_New, compatible_subtypes=None):
    if compatible_subtypes is None:
        compatible_subtypes = [subtype]
    if subtype == messaging.ParamValueType.BoundedFloatType:
        compatible_subtypes.append(messaging.ParamValueType.floatType)
        compatible_subtypes.append(messaging.ParamValueType.int32Type)
    if subtype == messaging.ParamValueType.floatType:
        compatible_subtypes.append(messaging.ParamValueType.BoundedFloatType)
    if subtype == messaging.ParamValueType.int32Type:
        compatible_subtypes.append(messaging.ParamValueType.BoundedFloatType)
    return messaging.LocalNodeInput(input, subtype, schedType, compatible_subtypes)

Unpickle_Filters = {
    'LocalNodeInput' : makeNewLocalNodeInput,
}

class FilterUnpickler(pickle.Unpickler):
    def __init__(self, file):
        pickle.Unpickler.__init__(self, file)

    def load_reduce(self):
        stack = self.stack
        args = stack.pop()
        func = stack[-1]
        if func.__name__ in Unpickle_Filters:
            warning('filtering pickle load of %s(%s)' % (func.__name__, args))
            value = Unpickle_Filters[func.__name__](*args)
        else:
            value = func(*args)
        stack[-1] = value
    pickle.Unpickler.dispatch[pickle.REDUCE] = load_reduce


#
# SIMILARLY, don't rely on these filters to image pipeline node parameters
# either: they're designed to scale into a progressive migrations system
# (multiple filters could easily be applied progressively), but performance
# would necessarily get pretty awful pretty quickly
#

def filterPercentileNodeParameters(params_in):
    params_out = {}
    for param, value in params_in.items():
        param_key = param.input if isinstance(param, messaging.LocalNodeInput) else param
        if param_key == 'percentile' and isinstance(value, float):
            params_out[param] = messaging.BoundedFloat(value, 0, 100, messaging.BoundedFloatType.Clamps)
        else:
            params_out[param] = value;
    return params_out

def filterClampXNodeParameters(params_in):
    params_out = {}
    range_max = None
    range_min = None
    for param, value in params_in.items():
        param_key = param.input if isinstance(param, messaging.LocalNodeInput) else param
        if param_key == 'Max':
            range_max = value
        elif param_key == 'Min':
            range_min = value
        else:
            params_out[param] = value;
    if range_min is not None and range_max is not None:
        params_out['Range'] = messaging.Range(float(range_min), float(range_max))
    return params_out

def filterLevelsNodeParameters(params_in):
    params_out = {}
    for param, value in params_in.items():
        param_key = param.input if isinstance(param, messaging.LocalNodeInput) else param
        if param_key == 'black level' and isinstance(value, int):
            params_out[param] = messaging.BoundedFloat(float(value), 0, 255, messaging.BoundedFloatType.Clamps)
        elif param_key == 'white level' and isinstance(value, int):
            params_out[param] = messaging.BoundedFloat(float(value), 0, 255, messaging.BoundedFloatType.Clamps)
        else:
            params_out[param] = value;
    return params_out

def filterGuiOutputNodeParameters(params_in):
    params_out = {}
    for param, value in params_in.items():
        param_key = param.input if isinstance(param, messaging.LocalNodeInput) else param
        if param_key == 'jpeg quality' and isinstance(value, int):
            params_out[param] = messaging.BoundedFloat(float(value), 0, 100, messaging.BoundedFloatType.Clamps)
        else:
            params_out[param] = value;
    return params_out


NodeParam_Filters = {
    messaging.NodeType.Percentile : filterPercentileNodeParameters,
    #messaging.NodeType.ClampInt   : filterClampXNodeParameters,
    #messaging.NodeType.ClampFloat : filterClampXNodeParameters,
    messaging.NodeType.Levels     : filterLevelsNodeParameters,
    messaging.NodeType.GuiOutput  : filterGuiOutputNodeParameters
}

class ConvenientObject(object):
    def __init__(self, model, node_id):
        self.model = model
        self.node_id = node_id

class ConvenientInput(ConvenientObject):
    def __init__(self, model, node_id, input_id):
        super(ConvenientInput, self).__init__(model, node_id)
        self.input_id = input_id
    def connect(self, co):
        self.model.addArcAsync(co.node_id, co.output_id, self.node_id, self.input_id)

class ConvenientOutput(ConvenientObject):
    def __init__(self, model, node_id, output_id):
        super(ConvenientOutput, self).__init__(model, node_id)
        self.output_id = output_id
    def connect(self, ci):
        self.model.addArcAsync(self.node_id, self.output_id, ci.node_id, ci.input_id)

class ConvenientParam(ConvenientObject):
    def __init__(self, model, node_id, param_id):
        super(ConvenientParam, self).__init__(model, node_id)
        self.param_id = param_id
    def set(self, value):
        self.model.setParameterAsync(self.node_id, self.param_id, value)

class ConvenientNode(ConvenientObject):
    def __init__(self, model, node_id, params, inputs, outputs):
        super(ConvenientNode, self).__init__(model, node_id)
        self.valid_params = set()
        self.valid_inputs = set()
        self.valid_outputs = set()
        for kv in params:
            self.valid_params.add(kv.key().input)
        for kv in inputs:
            self.valid_inputs.add(kv.key().input)
        for kv in outputs:
            self.valid_outputs.add(kv.key().output)
    def setParam(self, param, value):
        self.model.setParameterAsync(self.node_id, param, value)
    def connectOutput(self, output, other_node, input):
        if isinstance(other_node, ConvenientNode):
            other_node_id = other_node.node_id
        else:
            other_node_id = other_node
        self.checkOutput(output)
        self.model.addArcAsync(self.node_id, output, other_node_id, input) 
    def connectInput(self, input, other_node, output):
        if isinstance(other_node, ConvenientNode):
            other_node_id = other_node.node_id
        else:
            other_node_id = other_node
        self.check_input(input)
        self.model.addArcAsync(other_node_id, output, self.node_id, input) 
    def checkInput(self, id):
        if id not in self.valid_inputs:
            warning('unknown node input: %s (node:%s)' % (id, self.node_id))
    def checkOutput(self, id):
        if id not in self.valid_outputs:
            warning('unknown node output: %s (node:%s)' % (id, self.node_id))
    def checkParam(self, id):
        if id not in self.valid_params:
            warning('unknown node parameter: %s (node:%s)' % (id, self.node_id))
    def remove(self):
        return self.x() 
    def output(self, id):
        return self.o(id)
    def input(self, id):
        return self.i(id)
    def param(self, id):
        return self.p(id)
    def o(self, id):
        self.checkOutput(id)
        return ConvenientOutput(self.model, self.node_id, id) 
    def i(self, id):
        self.checkInput(id)
        return ConvenientInput(self.model, self.node_id, id)
    def p(self, id):
        self.checkParam(id)
        return ConvenientParam(self.model, self.node_id, id)
    def x(self):
        return self.model.removeSynchronous(self.node_id)


class Model(messaging.MessageObserver):
    '''The Model class represents an image pipeline, and provides methods to manipulate its state.'''
    def __init__(self, node, pipeline_name = "default"):
        messaging.MessageObserver.__init__(self)
        self.__node = node

        self.pipeline_name = pipeline_name

        self.description_ready_condition = threading.Condition()
        self.graph_description = None

        self.node_added_condition = threading.Condition()
        self.node_added_wait_for = None
        self.node_added = None

        self.node_removed_condition = threading.Condition()
        self.node_removed_wait_for = None
        self.node_removed = None

        self.parameter_set_condition = threading.Condition()
        self.parameter_set_wait_for = None
        self.parameter_set = None

        self.arc_added_condition = threading.Condition()
        self.arc_added_wait_for = None
        self.arc_added = None

        node.addObserver(self)
        node.subMessage(messaging.NodeAddedMessage())
        node.subMessage(messaging.NodeRemovedMessage())
        node.subMessage(messaging.NodeParametersMessage())
        node.subMessage(messaging.GraphDescriptionMessage())
        node.subMessage(messaging.ArcAddedMessage())

    def clear(self):
        '''Remove all nodes from the pipeline.'''
        debug('Clearing pipeline %s' %(self.pipeline_name))
        self.send(messaging.ClearPipelineMessage(self.pipeline_name))

    def save(self, picklefname, timeout=3.0):
        '''Save the pipeline to file 'picklefname'.'''
        with open(picklefname, 'wb') as outf:
            saved = self.get(timeout)
            pickle.dump(saved, outf)
            
    def pause(self):
        self.send(messaging.SetPipelineStateMessage(self.pipeline_name, messaging.PipelineState.Pause))
            
    def play(self):
        self.send(messaging.SetPipelineStateMessage(self.pipeline_name, messaging.PipelineState.Play))
    
    def load(self, picklefname, timeout=3.0):
        '''Load the pipeline from 'picklefname', this will clear any existing pipeline.'''
        with open(picklefname, 'rb') as inf:
            #saved = pickle.load(inf)
            saved = FilterUnpickler(inf).load()
            self.set(saved, timeout)
    
    @staticmethod
    def loadFile(inf):
        '''Load a pipeline file from disk, and return its contents. This does not manipulate the image pipeline.'''
        #saved = pickle.load(inf)
        saved = FilterUnpickler(inf).load()
        return saved
    
    # !!! TODO: this should probably be a classmethod
    def dumpFile(self, outf, state):
        '''Save a passed pipeline state to a file. This does not manipulate the image pipeline.'''
        pickle.dump(outf, state)

    def get(self, timeout=3.0):
        '''Grab the state from the image pipeline, save it and return it.'''
        graph = self.__getSynchronousGraphDescription(timeout)
        if graph is None:
            raise RuntimeError(
                'Could not get Description Message from the pipeline, is it running?'
            )
        s = State()
        # don't save parameter values that are derived from links:
        linked_inputs = []
        for id in graph.nodeOutputs.keys():
            outputlinks = graph.nodeOutputs[id]
            for output in outputlinks.keys():
                links = outputlinks[output]
                for link in links:
                    linked_inputs.append((link.node, link.input))

        for id in graph.nodeTypes.keys():
            s.nodes[id] = Node(id, int(graph.nodeTypes[id]))
        for id, pvps in graph.nodeParams.items():
            for param, value in pvps.items():
                if (id, param) in linked_inputs:
                    debug('not saving parameter %d:%s (linked)' % (id, param))
                else:
                    s.nodes[id].params[param] = fromNPV(value)
                    debug('%s = %s (%d)' % (param, str(value), value.which))
        for id in graph.nodeInputs.keys():
            inputlinks = graph.nodeInputs[id]
            for input in inputlinks.keys():
                inlink = inputlinks[input]
                s.nodes[id].inarcs[input] = (inlink.node, inlink.output)
        for id in graph.nodeOutputs.keys():
            outputlinks = graph.nodeOutputs[id]
            for output in outputlinks.keys():
                links = outputlinks[output]
                #print 'node', id, 'output', output, '->',
                s.nodes[id].outarcs[output] = []
                for link in links:
                    #print link.node, link.input, ":",
                    s.nodes[id].outarcs[output].append((link.node, link.input))
                #print
        return s
    
    def set(self, state, timeout=3.0, clear=True):
        '''Set the state of the image pipeline based on 'state'.'''
        nodeTypes = {node_id: messaging.NodeType(node.type) for (node_id, node) in state.nodes.iteritems()}
        nodeArcs = {node_id: {key: messaging.NodeOutput(output[0], output[1], messaging.OutputType(0), 0) for key, output in node.inarcs.iteritems()} for (node_id, node) in state.nodes.iteritems()}
        nodeParams = {node_id: node.params for (node_id, node) in state.nodes.iteritems()}
        self.send(messaging.SetPipelineMessage(self.pipeline_name, nodeTypes, nodeArcs, nodeParams))
        """
        debug("Setting pipeline %s." %(self.pipeline_name))
        if clear: self.clear()
        id_map = {}
        node_map = {}
        # first ensure all nodes are present
        for old_id, node in state.nodes.items():
            try:
                id_map[old_id] = self.addSynchronous(node.type, timeout)
            except RuntimeError, e:
                error(str(e) + ": attempted to add node %s" % node.type)
                debug('attempting to continue...')
                id_map[old_id] = None
            node_map[old_id] = node
            #print id_map[old_id], 'is new id for', old_id, node.type, node.params
        connected_inputs = set()
        for old_id, node in state.nodes.items():
            for input in node.inarcs.keys():
                if isinstance(input, messaging.LocalNodeInput):
                    input_key = input.input
                else:
                    input_key = input
                (other, output) = node.inarcs[input]
                if other != 0 and id_map[other] is not None:
                    connected_inputs.add((old_id, input_key))
        # then set all parameter values (for non-connected parameters only)
        for old_id, node in state.nodes.items():
            id = id_map[old_id]
            if id is None:
                warning('skipping parameters for node that was not added: %s' %
                        str(node.params))
                continue
            if node.type in NodeParam_Filters:
                warning('filtering parameters of type %s node' % messaging.NodeType(node.type))
                params = NodeParam_Filters[node.type](node.params)
                debug('Filtered parameters:\n%s' % params)
            else:
                params = node.params
            for param in params.keys():
                if isinstance(param, messaging.LocalNodeInput):
                    param_key = param.input
                else:
                    param_key = param
                if (old_id, param_key) in connected_inputs:
                    debug('skipping setting connected parameter: %s:%s' % (old_id, param))
                    continue
                debug('%d.%s = %s (%s)' % (id, param, params[param], type(params[param])))
                try:
                    self.setParameterSynchronous(id, param_key, toNPV(params[param]), timeout)
                except RuntimeError, e:
                    error(str(e) + ": attempted to set parameter %s to %s" % ((id, param_key), params[param]))
                    debug('attempting to continue...')
        # finally add links
        for old_id, node in state.nodes.items():
            # strictly speaking only one of these should be necessary, since
            # arcs have two ends...
            id = id_map[old_id]
            if id is None:
                warning('skipping arcs for node that was not added: %s' %
                        str(node.inarcs))
                continue
            for input in node.inarcs.keys():
                if isinstance(input, messaging.LocalNodeInput):
                    input_key = input.input
                else:
                    input_key = input
                (other, output) = node.inarcs[input]
                if other != 0 and id_map[other] is not None:
                    try:
                        self.addArcSynchronous(id_map[other], output, id, input_key, timeout)
                    except RuntimeError, e:
                        error(str(e) + ': attempted to add arc %s --> %s' % ((id_map[other], output),(id, input)))
                        debug('attempting to continue...')
            #for output in node.outarcs.keys():
            #    (other, input) = node.outarcs[output]
            #    if other != 0:
            #        print 'set outarc:', id_map[id], output, other, input
            """
   
    def send(self, msg):
        '''Send a message to the pipeline group.'''
        self.__node.send(msg, "pipeline")
    

    def addNode(self, type, timeout=3.0):
        '''Return a Node object (corresponding to a node in the image pipeline) that can be used to manipulate the corresponding image pipeline node.'''
        return self.addSynchronous(type, timeout, return_class=True)

    def addSynchronous(self, type, timeout=3.0, return_class=False):
        '''Add a node, wait for it to be added, return the ID of the added node.
            
            Raises a RuntimeError if a timeout occurs.
            
            If return_class is set to true, then instead of just the node id, a
            class representing the image pipeline node is returned, with
            attributes initialised from the inputs, outputs and parameters of
            the node.

            addNode() calls this function with return_class=True
        '''
        debug('addSynchronous %d = %s' % (type, str(messaging.NodeType(type))))
        self.node_added_condition.acquire()
        self.node_added = None
        self.node_added_wait_for = messaging.NodeType(type)
        self.send(messaging.AddNodeMessage(
            self.pipeline_name,
            messaging.NodeType(type),
            messaging.NodeInputArcVec(),
            messaging.NodeOutputArcVec()
        ))
        self.node_added_condition.wait(timeout)
        if self.node_added is None:
            self.node_added_condition.release()
            raise RuntimeError('No response from pipeline %s, is it running?' %(self.pipeline_name))
        if return_class:
            r = ConvenientNode(
                self,
                self.node_added.nodeId,
                self.node_added.params,
                self.node_added.inputs,
                self.node_added.outputs
            )
        else:
            r = self.node_added.nodeId
            debug('Node id = %d' %(r,))
        self.node_added_condition.release()
        return r
        
    def removeSynchronous(self, node, timeout=3.0):
        '''Remove a node, wait for it to be removed. Raises a RuntimeError if a timeout occurs.'''
        debug('removeSynchronous %d' % (node,))
        self.node_removed_condition.acquire()
        self.node_removed = None
        self.node_removed_wait_for = node
        self.send(messaging.RemoveNodeMessage(
            self.pipeline_name,
            node
        ))
        self.node_removed_condition.wait(timeout)
        if self.node_removed is None:
            self.node_removed_condition.release()
            raise RuntimeError('No response from pipeline %s, is it running?' %(self.pipeline_name))
        r = self.node_removed.nodeId
        self.node_removed_condition.release()
        return None
    
    def setParameterSynchronous(self, node, param, value, timeout=3.0):
        '''Set a node parameter, wait for it to be set. Raises a RuntimeError if a timeout occurs.'''
        self.parameter_set_condition.acquire()
        self.parameter_set = None
        self.parameter_set_wait_for = (node, param)
        self.setParameterAsync(node, param, value)
        self.parameter_set_condition.wait(timeout)
        if self.parameter_set is None:
            self.parameter_set_condition.release()
            raise RuntimeError('No response from pipeline %s, is it running?' %(self.pipeline_name))
        self.parameter_set_condition.release()
        return None
    
    def setParameterAsync(self, node, param, value):
        msg = messaging.SetNodeParameterMessage(
            self.pipeline_name, node, param, value
        )
        debug('set parameter: %s' % msg)        
        self.send(msg)


    def composeArc(self, src, out, dst, inp):
        fr = messaging.NodeOutput()
        fr.node = src
        fr.output = out
        fr.type = messaging.OutputType.Image
        to = messaging.NodeInput()
        to.node = dst
        to.input = inp
        return fr, to
    
    def addArcSynchronous(self, src, out, dst, inp, timeout=3.0):
        self.arc_added_condition.acquire()
        self.arc_added = None
        fr, to = self.composeArc(src, out, dst, inp)
        self.arc_added_wait_for = (fr, to)
        self.addArcAsync(src, out, dst, inp)
        self.arc_added_condition.wait(timeout)
        if self.arc_added is None:
            self.arc_added_condition.release()
            raise RuntimeError('No response from pipeline %s, is it running?' %(self.pipeline_name))
        self.arc_added_condition.release()
        return None
    
    def addArcAsync(self, src, out, dst, inp):
        fr, to = self.composeArc(src, out, dst, inp)
        debug('add arc: %s --> %s' % (fr, to))        
        self.send(messaging.AddArcMessage(self.pipeline_name, fr, to))

    # Overloads for observing pipeline-related messages:

    def checkName(self, msg):
        if msg.pipelineName != self.pipeline_name:
            #debug('ignoring message (name %s != %s)' % 
            #    (msg.pipelineName, self.pipeline_name)
            #)
            return False
        return True

    def onNodeAddedMessage(self, m):
        if not self.checkName(m): return
        if self.node_added_wait_for is not None and \
           self.node_added_wait_for == m.nodeType:
            self.node_added_wait_for = None
            self.node_added_condition.acquire()
            self.node_added = copy.deepcopy(m)
            self.node_added_condition.notify()
            self.node_added_condition.release()
        else:
            pass
            #warning('ignoring message about unknown node being added')

    def onNodeRemovedMessage(self, m):
        if not self.checkName(m): return
        if self.node_removed_wait_for is not None and \
           self.node_removed_wait_for == m.nodeId:
            self.node_removed_wait_for = None
            self.node_removed_condition.acquire()
            self.node_removed = copy.deepcopy(m)
            self.node_removed_condition.notify()
            self.node_removed_condition.release()
        else:
            pass
            #warning('ignoring message about unknown node being removed')

    def onNodeParametersMessage(self, m):
        if not self.checkName(m): return
        if self.parameter_set_wait_for is not None and \
           m.nodeId == self.parameter_set_wait_for[0] and\
           self.parameter_set_wait_for[1] in map(lambda x: x.input, m.params.keys()):
            self.parameter_set_wait_for = None
            self.parameter_set_condition.acquire()
            self.parameter_set = copy.deepcopy(m)
            self.parameter_set_condition.notify()
            self.parameter_set_condition.release()
        else:
            pass
            #warning('ignoring message about unknown parameter being set')

    def onGraphDescriptionMessage(self, m):
        if not self.checkName(m): return
        self.description_ready_condition.acquire()
        self.graph_description = copy.deepcopy(m)
        self.description_ready_condition.notify()
        self.description_ready_condition.release()
    
    def onArcAddedMessage(self, m):
        if not self.checkName(m): return
        # oops, shouldn't have called a message field, 'from'
        if self.arc_added_wait_for is not None and \
           getattr(m, 'from').node == self.arc_added_wait_for[0].node and \
           getattr(m, 'from').output == self.arc_added_wait_for[0].output and \
           m.to.node == self.arc_added_wait_for[1].node and \
           m.to.input == self.arc_added_wait_for[1].input:
            self.arc_added_wait_for = None
            self.arc_added_condition.acquire()
            self.arc_added = copy.deepcopy(m)
            self.arc_added_condition.notify()
            self.arc_added_condition.release()
        else:
            pass
            #warning('ignoring message about unknown arc being added')

    #def onArcRemovedMessage(self, m):
    #    if not self.checkName(m): return
    #    #print m
    #    pass

    #def onStatusMessage(self, m):
    #    if not self.checkName(m): return
    #    #print m
    #    pass

    #def onInputStatusMessage(self, m):
    #    if not self.checkName(m): return
    #    #print m
    #    pass
    
    #def onOutputStatusMessage(self, m):
    #    if not self.checkName(m): return
    #    #print m
    #    pass
    
    #def onGuiImageMessage(self, m):
    #    if not self.checkName(m): return
    #    #print m
    #    pass

    def __getSynchronousGraphDescription(self, timeout=3.0):
        self.description_ready_condition.acquire()
        self.graph_description = None
        self.send(messaging.GraphRequestMessage(self.pipeline_name));
        self.description_ready_condition.wait(timeout)
        self.description_ready_condition.release()
        return self.graph_description
                

