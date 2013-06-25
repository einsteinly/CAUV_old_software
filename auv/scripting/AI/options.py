from cauv.debug import debug, info, warning, error
from utils.boundedtypes import BoundedBase
from utils.conv import BoostMapToDict
import collections
import cauv.messaging

cmd_representable_types = (int, float, str, bool, BoundedBase)
option_representable_types = (int, float, str, bool, cauv.messaging.Colour, BoundedBase)

def bool_constructor(string):
    if string.lower() in {"true", "yes", "y", "on", "1"}:
        return True
    if string.lower() in {"false", "no", "n", "off", "0"}:
        return False
    raise RuntimeError("Didn't understand {} as a boolean option".format(string))

def flatten_dict(opt_dict):
    flat_dict = {}
    for key, value in opt_dict.items():
        if isinstance(value, dict):
            flat_dict.update({key + "/" + k : v for k,v in flatten_dict(value).items()})
        else:
            flat_dict[key] = value
    return flat_dict

def flatten_dict_with_meta(opt_dict):
    flat_dict = {}
    for key, value_with_meta in opt_dict.items():
        if isinstance(value_with_meta.value, dict):
            suboptions = {k: OptionWithMeta(v, value_with_meta.dynamic, value_with_meta.docstring) for k,v in value_with_meta.value.iteritems()}
            flat_dict.update({key + "/" + k : v for k,v in flatten_dict_with_meta(suboptions).items()})
        else:
            flat_dict[key] = value_with_meta
    return flat_dict

def unflatten_dict(flat_dict):
    opt_dict = collections.defaultdict(dict)
    for key, value in flat_dict.items():
        key_parts = key.split("/", 1)
        if len(key_parts) == 2:
            opt_dict[key_parts[0]][key_parts[1]] = value
            opt_dict[key_parts[0]] = unflatten_dict(opt_dict[key_parts[0]])
        else:
            opt_dict[key] = value
    return dict(opt_dict)

def dict_to_cmd_opts(flat_dict):
    cmd_opts = []
    for key, value in flat_dict.iteritems():
        cmd_opts.append("--{}".format(key))
        cmd_opts.append(str(value))
    return cmd_opts

class OptionWithMeta(object):
    """
    When used inside an option class, this will behave like the value, but holds additional meta info:
    dynamic - suggests that it is safe to modify this option while running (not enforced)
    docstring - add docstring to option
    opt_type - a function/class that processes a value when it is set.
    """
    def __init__(self, value, dynamic=False, docstring="", opt_type="", units=""):
        self.value = value
        self.dynamic = dynamic
        self.docstring = docstring
        self.opt_type = opt_type
        self.units = units
    
    def to_dict(self, representable_types = option_representable_types):
        if isinstance(self.value, representable_types):
            return self
        else:
            try:
                #TODO deal with opt_type argument
                return OptionWithMeta(self.value.to_dict(), self.dynamic, self.docstring, units=self.units)
            except AttributeError:
                raise TypeError("Could not represent value as dict.")
            
    def to_boost(self):
        if hasattr(self.value, "asParamValue"):
            value_as_param = self.value.asParamValue()
        else:
            value_as_param = cauv.messaging.ParamValue.create(self.value)
        return cauv.messaging.ParamWithMeta(value_as_param,self.dynamic,self.docstring,self.units)
        
class Options(object):
    def __getattribute__(self, attr):
        value = super(Options, self).__getattribute__(attr)
        if isinstance(value, OptionWithMeta):
            return value.value
        return value
    
    def iteroptions(self):
        for (attr, value) in self.__dict__.iteritems():
            if attr.startswith("_"):
                continue
            yield (attr, value)
    
    def __getrawattr__(self, attr):
        return super(Options, self).__getattribute__(attr)
        
    def __setattr__(self, attr, value):
        try:
            if isinstance(self.__getrawattr__(attr), OptionWithMeta):
                if self.__getrawattr__(attr).opt_type:
                    self.__getrawattr__(attr).value = self.__getrawattr__(attr).opt_type(value)
                    return
                self.__getrawattr__(attr).value = value
                return
            if hasattr(getattr(self, attr), "setValue"):
                getattr(self, attr).setValue(value)
                return
        except AttributeError:
            pass
        super(Options, self).__setattr__(attr, value)
    
    def to_dict(self, representable_types = option_representable_types):
        """Returns a dict with string keys and representable values (without meta data) or further
        dicts containing keys and representable values. Types which are not
        directly representable should also implement to_dict() with the same
        contract"""
        opts = {}
        for option, raw_value in self.iteroptions():
            if isinstance(raw_value, OptionWithMeta):
                value = raw_value.value
            else:
                value = raw_value
            if isinstance(value, representable_types):
                opts[option] = value
            else:
                try:
                    opts[option] = value.to_dict()
                except AttributeError:
                    warning("Value {} ({}) cannot be represented".format(option, value))
        return opts
    
    
    def to_dict_with_meta(self, representable_types = option_representable_types):
        """Returns a dict with string keys and representable values (with meta data) or further
        dicts containing keys and representable values. Types which are not
        directly representable should also implement to_dict() with the same
        contract"""
        opts = {}
        for option, raw_value in self.iteroptions():
            if isinstance(raw_value, OptionWithMeta):
                value = raw_value
            else:
                value = OptionWithMeta(raw_value)
            try:
                opts[option] = value.to_dict(representable_types)
            except TypeError:
                warning("Value {} ({}) cannot be represented".format(option, value.value))
        return opts

    def from_dict(self, opt_dict):
        """Applies the options in opt_dict (without meta data) to the object.""" 
        for option, value in opt_dict.iteritems():
            if isinstance(value, dict):
                getattr(self, option).from_dict(value)
            else:
                setattr(self, option, value)

    def to_flat_dict(self):
        return flatten_dict(self.to_dict())
    
    def to_flat_dict_with_meta(self):
        return flatten_dict_with_meta(self.to_dict_with_meta())
    
    def to_boost_dict(self):
        return {k: v.to_boost() for k, v in self.to_flat_dict_with_meta().items()}
    
    def from_boost_dict(self, boost_map):
        return self.from_flat_dict(BoostMapToDict(boost_map))

    def from_flat_dict(self, flat_opt_dict):
        return self.from_dict(unflatten_dict(flat_opt_dict))

    def to_cmd_opts(self):
        return dict_to_cmd_opts(flatten_dict(self.to_dict(cmd_representable_types)))

def add_options_to_argparse(parser, options):
    for option, value_with_meta in sorted(options.to_flat_dict_with_meta().iteritems()):
        arg_type = value_with_meta.opt_type
        if arg_type == '':
            arg_type = type(value_with_meta.value)
        if arg_type == bool:
            arg_type = bool_constructor
        parser.add_argument("--" + option, default = value_with_meta.value, type = arg_type, help=value_with_meta.docstring)

