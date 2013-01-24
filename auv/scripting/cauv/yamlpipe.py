#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import messaging as msg
import pipeline as pipe

import re
import yaml
import struct
import collections

class PipelineDumper(yaml.Dumper):
    def __init__(self, pipe_state, *args, **kargs):
        yaml.Dumper.__init__(self, *args, **kargs)
        self.node_ids = collections.defaultdict(int)
        self.pipeline_state = pipe_state
    def generate_anchor(self, node):
        node_type = node.tag.split('/')[-1]
        self.node_ids[node_type] += 1
        return "{}_{}".format(node_type, self.node_ids[node_type])

class PipelineLoader(yaml.Loader):
    def __init__(self, *args, **kargs):
        yaml.Loader.__init__(self, *args, **kargs)
        self.pipeline_state = pipe.State()
        self.n_nodes = 0

def bounded_float_representer(dumper, b_float):
    return dumper.represent_scalar(u"!BoundedFloat/{}".format(b_float.type.name), u"{} ({}-{})".format(b_float.value, b_float.min, b_float.max))

#Eeeeyup...
bounded_float_re = re.compile('(?P<value>[0-9.]+)\s+\\((?P<min>[0-9.]+)\s*-\s*(?P<max>[0-9.]+)\\)')
def bounded_float_constructor(loader, tag_suffix, data):
    description = loader.construct_scalar(data)
    value, min_, max_  = bounded_float_re.match(description).groups()
    type_ = msg.BoundedFloatType.names[tag_suffix]
    return msg.BoundedFloat(float(value), float(min_), float(max_), type_)

def colour_representer(dumper, colour):
    #XXX hack since array values can't be accessed from python sensibly
    #Why do we even have arrays anyway? There should be a ban on new datatypes
    #being implemented at SAUCE-E
    colourvals = struct.unpack('<ffff', colour.values.__reduce__()[1][0])
    return dumper.represent_scalar(u"!colour/{}".format(colour.type), " ".join((u"{:.4}",)*4).format(*colourvals))

def colour_constructor(loader, tag_suffix, data):
    description = loader.construct_scalar(data)
    colourvals = [float(x) for x in description.split()]
    type_ = msg.ColourType.names[tag_suffix]
    return msg.Colour(type_, colourvals)

def node_representer(dumper, node):
    values = {}
    for _input, param in node.params.iteritems():
        values[_input.input] = param
    for _input, inarc in node.inarcs.iteritems():
        if inarc[0]:
            try:
                del values[_input.input]
            except KeyError:
                pass
            values["{}<-{}".format(_input.input, inarc[1])] = dumper.pipeline_state.nodes[inarc[0]]
    value_list = list(values.items())
    value_list.sort(key=lambda x: ('<-' in x[0], x[0]))
    return dumper.represent_mapping(u"!node/{}".format(msg.NodeType.values[node.type].name), value_list)

def node_constructor(loader, tag_suffix, data):
    type_ = msg.NodeType.names[tag_suffix]
    values = loader.construct_mapping(data)
    loader.n_nodes += 1
    node = pipe.Node(loader.n_nodes, type_)
    for _input, inval in values.iteritems():
        if '<-' in _input:
            _input, output = _input.split('<-')
            #pretty sure this fakery of inputschedtype and compatiblesubtypes is OK. 
            #The image pipeline leaks way too much of its internal details into the 
            #messaging system.
            local_input = msg.LocalNodeInput(_input, 0, msg.InputSchedType.Must_Be_New, [])
            node.inarcs[local_input] = (inval.id, output)
        else:
            local_input = msg.LocalNodeInput(_input, 0, msg.InputSchedType.Must_Be_New, [])
            node.params[local_input] = inval
    loader.pipeline_state.nodes[node.id] = node
    return node


yaml.add_representer(msg.BoundedFloat, bounded_float_representer)
yaml.add_representer(msg.Colour, colour_representer)
yaml.add_representer(pipe.Node, node_representer)
yaml.add_multi_constructor('!BoundedFloat/', bounded_float_constructor)
yaml.add_multi_constructor('!colour/', colour_constructor)
yaml.add_multi_constructor("!node/", node_constructor)

def PipelineDumperFactory(model): #urg
    def constructor(*args, **kargs):
        return PipelineDumper(model, *args, **kargs)
    return constructor

def dump(outf, state):
    toplevel_nodes = []
    #must... resist... temptation... to put in a list (in)comprehension
    for node in state.nodes.values():
        #toplevel nodes are any nodes with no output arcs
        if not any((any(o[0] for o in a) for a in node.outarcs.values())):
            toplevel_nodes.append(node)
    yaml.dump(toplevel_nodes, stream = outf, Dumper = PipelineDumperFactory(state))

def load(inf):
    loader = PipelineLoader(inf)
    loader.get_single_data()
    loaded_state = loader.pipeline_state
    for node in loaded_state.nodes.values():
        for node_id, inarc in node.inarcs.values():
            #similarly to LocalNodeInput above, type and subtype are just faked
            output = msg.LocalNodeOutput(inarc, msg.OutputType.Image, 0)
            o_node = loaded_state.nodes[node_id]
            if output not in o_node.outarcs:
                o_node.outarcs[output] = []
            o_node.outarcs[output].append((node.id, inarc))
    return loaded_state
