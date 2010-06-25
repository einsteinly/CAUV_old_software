import messaging
import threading

class Node:
    def __init__(self, id, type, params = {}, inarcs = {}, outarcs = {}):
        self.type = type
        self.params = params   # param : value
        self.inarcs = inarcs   # input : (node id, output)
        self.outarcs = outarcs # output : (node id, input)

class State:
    def __init__(self):
        self.nodes = {} # id : Node

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

    def get(self):
        graph = self.__getSynchronousGraphDescription()
        if graph is None:
            raise RuntimeError(
                'Could not get Description Message from the pipeline, is it running?'
            )
        s = State()
        for id in graph.nodeTypes.keys():
            s.nodes[id] = Node(id, graph.nodeTypes[id])
        for id in graph.nodeParams.keys():
            values = graph.nodeParams[id]
            for param in values.keys():
                print 'parameter', param, 'value:', values[param]
                s.nodes[id].params[param] = values[param]
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
    
    def set(self, state):
        self.clear()
        id_map = {}
        # first ensure all nodes are present
        for id, node in state.nodes.items():
            id_map[id] = self.addSynchronous(node.type)
        # then set all parameter values
        for id, node in state.nodes.items():
            for param in node.params.keys():
                #print 'set parameter', node, param, '=', node.params[param]
                self.setParameterSynchronous(id_map[id], param, node.params[param])
        # finally add links
        for id, node in state.nodes.items():
            # strictly speaking only one of these should be necessary, since
            # arcs have two ends...
            for input in node.inarcs.keys():
                (other, output) = node.inarcs[input]
                #print 'set inarc:', id_map[id], input, other, output
                self.addArcSynchronous(id_map[id], input, other, output)
            #for output in node.outarcs.keys():
            #    (other, input) = node.outarcs[output]
            #    print 'set outarc:', id_map[id], output, other, input
   
    def send(self, msg):
        # send to pipeline via self.__node
        self.__node.send(msg, "pipeline")


    def addSynchronous(self, type, timeout=3.0):
        self.node_added_condition.acquire()
        self.node_added = None        
        self.send(messaging.AddNodeMessage(
            type, messaging.NodeInputArcVec(), messaging.NodeOutputArcVec()
        ))
        self.node_added_condition.wait(timeout)
        if self.node_added is None:
            self.node_added_condition.release()
            raise RuntimeError('No reponse from pipeline, is it running?')
        self.node_added_condition.release()
        return self.node_added.nodeId
    
    def setParameterSynchronous(self, node, param, value, timeout=3.0):
        self.parameter_set_condition.acquire()
        self.parameter_set = None
        self.send(messaging.SetNodeParameterMessage(
            node, param, value
        ))
        self.parameter_set_condition.wait(timeout)
        if self.parameter_set is None:
            self.parameter_set_condition.release()
            raise RuntimeError('No reponse from pipeline, is it running?')
        self.parameter_set_condition.release()
        return None
    
    def addArcSynchronous(self, src, out, dst, inp, timeout=3.0):
        self.arc_added_condition.acquire()
        self.arc_added = None
        fr = messaging.NodeOutput()
        fr.node = src
        fr.output = out
        # TODO!!!!!!
        fr.type = messaging.OutputType.Image
        # !!!!!!
        to = messaging.NodeInput()
        to.node = dst
        to.input = inp
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
        print m 

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
        print m 

    def onStatusMessage(self, m):
        print m 

    def onInputStatusMessage(self, m):
        print m 
    
    def onOutputStatusMessage(self, m):
        print m 
    
    #def onGuiImageMessage(self, m):
    #    print m 

    def __getSynchronousGraphDescription(self, timeout=3.0):
        self.description_ready_condition.acquire()
        self.graph_description = None
        self.send(messaging.GraphRequestMessage());
        print '!!\t\twaiting on condition...'
        self.description_ready_condition.wait(timeout)
        print '!!\t\tnotified'
        self.description_ready_condition.release()
        return self.graph_description
                

