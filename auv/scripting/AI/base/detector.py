from cauv import messaging

from AI.base.option import aiOptions

class aiDetectorOptions(aiOptions):
    pass
        
class aiDetector(messaging.MessageObserver):
    debug_values = []
    def __init__(self, node, opts):
        messaging.MessageObserver.__init__(self)
        self.options = opts
        self.node = node
        self.node.addObserver(self)
        self.detected = False
        self._detected_past = False
    def request_pl(self, name):
        raise NotImplementedError("pipeline management changed")
    def drop_pl(self, name):
        raise NotImplementedError("pipeline management changed")
    def drop_all_pl(self):
        raise NotImplementedError("pipeline management changed")
    def process(self):
        """
        This should define a method to do any intensive (ie not on message) processing
        """
        pass
    def set_option(self, option_name, option_value):
        setattr(self.options, option_name, option_value)
        self.optionChanged(option_name)
    def get_debug_values(self):
        debug = {}
        error_attrs = []
        for key_str in self.debug_values:
            keys = key_str.split('.')
            value = self
            try:
                for key in keys:
                    value = getattr(value, key)
                #make sure that we can transmit it
                try:
                    value = messaging.ParamValue.create(value)
                except TypeError:
                    value = messaging.ParamValue.create(debug_converters[type(value)](value))
            except Exception:
                error_attrs.append(key_str)
                continue
            debug[key_str] = value
        if error_attrs:
            warning("Could not get/encode attributes %s, skipping from debug value report" %str(error_attrs))
        return debug
    def log(self, message):
        debug(message)
        try:
            self.node.send(messaging.AIlogMessage(message), "ai")
        except:
            error('Error sending high-level log message')
            error(traceback.format_exc().encode('ascii','ignore'))
    def die(self):
        self.node.removeObserver(self)
    def optionChanged(self, option_name):
        pass