#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from cauv.debug import debug, info, error, warning

class aiOptionsBase(type):
    def __new__(cls, name, bases, attrs):
        new_attrs = {'_option_classes':{}, '_pipelines':tuple()}
        #add in 'inherited' options from parent class
        for base in bases:
            for key, attr in base.__dict__.iteritems():
                #but don't overwrite values
                if not key in attrs:
                    attrs[key] = attr
        #filter out pipeline info and set (use tuple to avoid overriding if possible)
        if 'Meta' in attrs:
            meta_data = attrs.pop('Meta')
            if hasattr(meta_data, 'pipelines'):
                attrs['_pipelines'] = tuple(meta_data.pipelines)
        for key, value in attrs.iteritems():
            #don't process 'system' values
            if not key[0] == '_':
                #if user defines a type for the option, force the type, and store type in meta (assume that this type is transmittable)
                if isinstance(value, tuple) and len(value)==2 and callable(value[1]):
                    new_attrs[key] = value[1](value[0])
                    new_attrs['_option_classes'][key]=value[1]
                #else leave (if can be transmitted)
                elif isinstance(value, (int,str,float,bool)):
                    new_attrs[key] = value
                #else need to make sure don't try to transmit, so relabel
                else:
                    new_attrs['_not_transmittable_'+key] = value
                    if not callable(value):
                        warning('Option %s on %s will not appear as is not a valid type' %(key, name))
            else:
                new_attrs[key] = value
        new_cls = super(aiOptionsBase, cls).__new__(cls, name, bases, new_attrs)
        return new_cls
    def __getattr__(cls, attr):
        #make sure also searches _not_transmittable stuff
        return cls.__getattribute__(cls, '_not_transmittable_'+attr)
    def get_default_options(cls):
        return dict([item for item in cls.__dict__.iteritems() if item[0][0] != '_'])
    def get_default_options_as_params(cls):
        #make sure converted to params if needed
        options={}
        for key, attr in cls.__dict__.iteritems():
            if key in cls._option_classes and hasattr(cls._option_classes[key], 'asParamValue'):
                options[key] = cls._option_classes[key].asParamValue(attr)
            else: options[key] = attr
        return options
    
class aiOptions(object):
    __metaclass__ = aiOptionsBase
    def __init__(self, options={}):
        #set values to default
        self.__dict__.update(self.__class__.get_default_options()) #pylint: disable=E1101
        for opt, val in options.iteritems():
            setattr(self, opt, val)
    def __getattr__(self, attr):
        return self.__getattribute__('_not_transmittable_'+attr)
    def __setattr__(self, key, attr):
        #force type if specified in meta
        if key in self._option_classes:
            attr=self._option_classes[key](attr)
        object.__setattr__(self, key, attr)
    def get_options(self):
        return dict([item for item in self.__dict__.iteritems() if item[0][0] != '_'])
    def get_options_as_params(self):
        #make sure converted to params if needed
        options={}
        for key, attr in self.__dict__.iteritems():
            if key in self._option_classes and hasattr(self._option_classes[key], 'asParamValue'):
                options[key] = self._option_classes[key].asParamValue(attr)
            else: options[key] = attr
        return options