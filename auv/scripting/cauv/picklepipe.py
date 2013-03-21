from debug import debug, warning, error, info
import messaging

import pickle
import pipeline

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

def load(inf):
    '''Load a pipeline file from disk, and return its contents. This does not manipulate the image pipeline.'''
    saved = FilterUnpickler(inf).load()
    return saved

def dump(outf, state):
    '''Save a passed pipeline state to a file. This does not manipulate the image pipeline.'''
    pickle.dump(state, outf)
