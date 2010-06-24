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
        print graph
        s = State()
        print dir(graph)
        for id in graph.nodeTypes.keys():
            s.nodes[id] = Node(id, graph.nodeTypes[id])
        for id in graph.nodeParams.keys():
            values = graph.nodeParams[id]
            for param in values.keys():
                s.nodes[id].params[param] = values[param]
        for id in graph.nodeInputs.keys():
            inputlinks = graph.nodeInputs[id]
            for input in inputlinks.keys():
                s.nodes[id].inarcs[input] = inputlinks[input]
        for id in graph.nodeOutputs.keys():
            outputlinks = graph.nodeOutputs[id]
            for output in outputlinks.keys():
                links = outputlinks[output]
                print 'node', id, 'output', output, '->',
                for link in links:
                    print link,
                print
        return s
    
    def set(self, state):
        self.clear()
        id_map = {}
        # first ensure all nodes are present
        for id, node in state.nodes.items():
            id_map[id] = self.addSynchronous(node.type)
        # then set all parameter values
        for id, node in state.nodes.items():
            for param, value in node.params:
                self.setParameterSynchronous(id_map[id], param, value)
        # finally add links
        for id, node in state.nodes.items():
            # strictly speaking only one of these should be necessary...
            for input, (node_id, output) in node.inarcs:
                print input, node_id, output
                pass
            for output, (node_id, input) in node.outarcs:
                print output, node_id, input
                pass
   
    def send(self, msg):
        # send to pipeline via self.__node
        self.__node.send(msg, "pipeline")


    def addSynchronous(self, type):
        return None
    
    def setParameterSynchronous(self, param, value):
        return None

    # Overloads for observing pipeline-related messages:

    def onNodeAddedMessage(self, m):
        print m 

    def onNodeRemovedMessage(self, m):
        print m 

    def onNodeParametersMessage(self, m):
        print m 

    def onGraphDescriptionMessage(self, m):
        print 'received graph desciption:'
        print m
        self.description_ready_condition.acquire()
        self.graph_description = m
        self.description_ready_condition.notify()
        self.description_ready_condition.release()

    def onArcAddedMessage(self, m):
        print m 

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
        self.graph_description = None
        self.description_ready_condition.acquire()
        self.send(messaging.GraphRequestMessage());
        print '!!\t\twaiting on condition...'
        self.description_ready_condition.wait(timeout)
        print '!!\t\tnotified'
        self.description_ready_condition.release()
        return self.graph_description
                

