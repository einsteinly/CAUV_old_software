from cauv.debug import debug, info, warning, error
import collections
import cauv.messaging

cmd_representable_types = (int, float, str, bool)
option_representable_types = (int, float, str, bool, cauv.messaging.Colour)

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

class Options(object):
    def to_dict(self, representable_types = option_representable_types):
        """Returns a dict with string keys and representable values or further
        dicts containing keys and representable values. Types which are not
        directly representable should also implement to_dict() with the same
        contract"""
        opts = {}
        for option, value in vars(self).items():
            if isinstance(value, representable_types):
                opts[option] = value
            else:
                try:
                    opts[option] = value.to_dict()
                except AttributeError:
                    warning("Value {} ({}) cannot be represented".format(option, value))
        return opts

    def from_dict(self, opt_dict):
        """Applies the options in opt_dict to the object.""" 
        for option, value in opt_dict.iteritems():
            if isinstance(value, dict):
                getattr(self, option).from_dict(value)
            else:
                setattr(self, option, value)

    def to_flat_dict(self):
        return flatten_dict(self.to_dict())

    def from_flat_dict(self, flat_opt_dict):
        return self.from_dict(unflatten_dict(flat_opt_dict))

    def to_cmd_opts(self):
        return dict_to_cmd_opts(flatten_dict(self.to_dict(cmd_representable_types)))

def add_options_to_argparse(parser, options):
    opt_dict = options.to_dict()
    for option, value in sorted(flatten_dict(opt_dict).iteritems()):
        arg_type = type(value)
        if arg_type == bool:
            arg_type = bool_constructor
        parser.add_argument("--" + option, default = value, type = arg_type)

