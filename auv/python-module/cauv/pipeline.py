import messaging
from debug import debug, warning, error, info

import threading
import pickle

intParam = messaging.NodeParamValue.from_int32
stringParam = messaging.NodeParamValue.from_string
boolParam = messaging.NodeParamValue.from_bool
floatParam = messaging.NodeParamValue.from_float

def fromNPV(npv):
    if npv.which == messaging.ParamType.String:
        return npv.string
    elif npv.which == messaging.ParamType.Bool:
        return npv.bool
    elif npv.which == messaging.ParamType.Int32:
        return npv.int32
    elif npv.which == messaging.ParamType.Float:
        return npv.float
    else:
        raise RuntimeError('Unknown type in NodeParamValue')

def toNPV(value):
    r = None
    # caveat viator: isinstance(bool(...), int) == True
    if isinstance(value, bool):
        r = messaging.NodeParamValue.from_bool(value)
    elif isinstance(value, int) or isinstance(value, long):
        r = messaging.NodeParamValue.from_int32(value)
    elif isinstance(value, float):
        r = messaging.NodeParamValue.from_float(value)
    elif isinstance(value, str):
        r = messaging.NodeParamValue.from_string(value)
    else:
        raise RuntimeError('Cannot pack "%s" as NodeParamValue' % type(value))
    return r
 
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

class Model(messaging.BufferedMessageObserver):
    def __init__(self, node):
        messaging.BufferedMessageObserver.__init__(self)
        self.__node = node

        self.description_ready_condition = threading.Condition()
        self.graph_description = None

        self.node_added_condition = threading.Condition()
        self.node_added = None

        self.parameter_set_condition = threading.Condition()
        self.parameter_set = None

        self.arc_added_condition = threading.Condition()
        self.arc_added = None

        node.join("pl_gui")
        node.addObserver(self)

    def clear(self):
        self.send(messaging.ClearPipelineMessage())

    def save(self, picklefname, timeout=3.0):
        with open(picklefname, 'wb') as outf:
            saved = self.get(timeout)
            pickle.dump(saved, outf)
    
    def load(self, picklefname, timeout=3.0):
        with open(picklefname, 'rb') as inf:
            saved = pickle.load(inf)
            self.set(saved, timeout)

    def get(self, timeout=3.0):
        graph = self.__getSynchronousGraphDescription(timeout)
        if graph is None:
            raise RuntimeError(
                'Could not get Description Message from the pipeline, is it running?'
            )
        s = State()
        for id in graph.nodeTypes.keys():
            s.nodes[id] = Node(id, int(graph.nodeTypes[id]))
        for id, pvps in graph.nodeParams.items():
            for param, value in pvps.items():
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
    
    def set(self, state, timeout=3.0):
        self.clear()
        id_map = {}
        node_map = {}
        # first ensure all nodes are present
        for old_id, node in state.nodes.items():
            id_map[old_id] = self.addSynchronous(node.type, timeout)
            node_map[old_id] = node
            #print id_map[old_id], 'is new id for', old_id, node.type, node.params
        # then set all parameter values
        for old_id, node in state.nodes.items():
            id = id_map[old_id]
            for param in node.params.keys():
                debug('%d.%s = %s' % (id, param, node.params[param]))
                self.setParameterSynchronous(id, param, toNPV(node.params[param]), timeout)
        # finally add links
        for old_id, node in state.nodes.items():
            # strictly speaking only one of these should be necessary, since
            # arcs have two ends...
            id = id_map[old_id]
            for input in node.inarcs.keys():
                (other, output) = node.inarcs[input]
                if other != 0:
                    self.addArcSynchronous(id_map[other], output, id, input, timeout)
            #for output in node.outarcs.keys():
            #    (other, input) = node.outarcs[output]
            #    if other != 0:
            #        print 'set outarc:', id_map[id], output, other, input
   
    def send(self, msg):
        # send to pipeline via self.__node
        self.__node.send(msg, "pipeline")


    def addSynchronous(self, type, timeout=3.0):
        debug('addSynchronous %d = %s' % (type, str(messaging.NodeType(type))))
        self.node_added_condition.acquire()
        self.node_added = None
        self.send(messaging.AddNodeMessage(
            messaging.NodeType(type), messaging.NodeInputArcVec(), messaging.NodeOutputArcVec()
        ))
        self.node_added_condition.wait(timeout)
        if self.node_added is None:
            self.node_added_condition.release()
            raise RuntimeError('No response from pipeline, is it running?')
        r = self.node_added.nodeId
        self.node_added_condition.release()
        return r
    
    def setParameterSynchronous(self, node, param, value, timeout=3.0):
        self.parameter_set_condition.acquire()
        self.parameter_set = None
        self.send(messaging.SetNodeParameterMessage(
            node, param, value
        ))
        self.parameter_set_condition.wait(timeout)
        if self.parameter_set is None:
            self.parameter_set_condition.release()
            raise RuntimeError('No response from pipeline, is it running?')
        self.parameter_set_condition.release()
        return None
    
    def addArcSynchronous(self, src, out, dst, inp, timeout=3.0):
        self.arc_added_condition.acquire()
        self.arc_added = None
        fr = messaging.NodeOutput()
        fr.node = src
        fr.output = out
        # TODO: type is currently ignored by pipeline when adding arcs: type is
        # checked anyway, and incorrect type rejected.
        fr.type = messaging.OutputType.Image
        to = messaging.NodeInput()
        to.node = dst
        to.input = inp
        debug('add arc: %s --> %s' % (fr, to))
        self.send(messaging.AddArcMessage(fr, to))
        self.arc_added_condition.wait(timeout)
        if self.arc_added is None:
            self.arc_added_condition.release()
            raise RuntimeError('No reponse from pipeline, is it running?')
        self.arc_added_condition.release()
        return None

    # Overloads for observing pipeline-related messages:

    def onNodeAddedMessage(self, m):
        self.node_added_condition.acquire()
        self.node_added = m
        self.node_added_condition.notify()
        self.node_added_condition.release()

    def onNodeRemovedMessage(self, m):
        #print m
        pass

    def onNodeParametersMessage(self, m):
        # TODO: be discriminating about whether this parameter message actually
        # corresponds to the one set in setParameterSynchronous
        self.parameter_set_condition.acquire()
        self.parameter_set = m
        self.parameter_set_condition.notify()
        self.parameter_set_condition.release()

    def onGraphDescriptionMessage(self, m):
        self.description_ready_condition.acquire()
        self.graph_description = m
        self.description_ready_condition.notify()
        self.description_ready_condition.release()

    def onArcAddedMessage(self, m):
        self.arc_added_condition.acquire()
        self.arc_added = m
        self.arc_added_condition.notify()
        self.arc_added_condition.release()

    def onArcRemovedMessage(self, m):
        #print m
        pass

    def onStatusMessage(self, m):
        #print m
        pass

    def onInputStatusMessage(self, m):
        #print m
        pass
    
    def onOutputStatusMessage(self, m):
        #print m
        pass
    
    #def onGuiImageMessage(self, m):
    #    #print m
    #    pass

    def __getSynchronousGraphDescription(self, timeout=3.0):
        self.description_ready_condition.acquire()
        self.graph_description = None
        self.send(messaging.GraphRequestMessage());
        #print '!!\t\twaiting on condition...'
        self.description_ready_condition.wait(timeout)
        #print '!!\t\tnotified'
        self.description_ready_condition.release()
        return self.graph_description
                

