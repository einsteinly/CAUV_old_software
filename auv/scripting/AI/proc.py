from cauv.debug import debug, warning, error, info
import cauv.messaging as msg
import cauv.control as control
import cauv.node as node
import cauv.yamlpipe
import cauv.pipeline
import utils.dirs

import os.path
import threading
import traceback

import time
import argparse
import collections

import options

class Proc(msg.MessageObserver):
    class DefaultOptions(options.Options):
        pass
    class DefaultState(object):
        pass
    class Debug(options.Options):
        pass
    
    def __init__(self, node_name):
        msg.MessageObserver.__init__(self)
        self.node = node.Node(node_name)
        self.auv = control.AUV_readonly(self.node)
        if hasattr(self, "report"):
            self._die_flag = threading.Event()
            self._report_frequency = 0.5
            self._reporting_thread = threading.Thread(target=self._report_loop)
            self._reporting_thread.start()
            
    def _report_loop(self):
        while not self._die_flag.wait(self._report_frequency):
            try:
                self.report()
            except Exception as e:
                error(traceback.format_exc().encode('ascii','ignore'))
        
    def start_listening(self):
        self.node.addObserver(self)

    def log(self, message):
        info("AI log: location {}, bearing {}, time {}: ".format(self.auv.getDepth(), self.auv.getBearing(), time.time()) + message)

    def map_pl_name(self, name):
        return "ai/{}_{}".format(self.options._task_name, name).lower()

    def load_pipeline(self, pipeline_name):
        pipe_file = os.path.join(utils.dirs.config_dir('pipelines'), pipeline_name) + ".yaml"
        info("Loading pipeline {}".format(pipe_file))
        with open(pipe_file) as pf:
            pipeline = cauv.yamlpipe.load(pf)
        model = cauv.pipeline.Model(self.node, self.map_pl_name(pipeline_name))
        pipeline.fixup_inputs()
        model.set(pipeline)
        
    def cleanup(self):
        self._die_flag.set()
        self.unload_pipeline("")
        info("Pipelines cleaned up")

    def unload_pipeline(self, pipeline_name):
        pl_delete = self.map_pl_name(pipeline_name)
        self.node.send(cauv.messaging.ClearPipelineMessage(pl_delete))

    @classmethod
    def get_options(cls):
        parser = argparse.ArgumentParser(description=cls.__doc__)
        #parser.add_argument("-s", "--state", help="File to read/write state from")
        #parser.add_argument("-o", "--options", help="File to read options from")
        parser.add_argument("-b", "--broadcast-status", help="Broadcast a success message after completion", action="store_true")
        parser.add_argument("-t", "--task-name", help="Name of task for which script is being run", default = cls.__name__)
        script_parser = parser.add_argument_group(title="Script Options",
                                                  description="Script specific options")

        default_options = cls.DefaultOptions()
        options.add_options_to_argparse(script_parser, default_options)
        try:
            opts = parser.parse_args(cls._test_args)
        except AttributeError:
            opts = parser.parse_args()

        default_options.from_flat_dict(vars(opts))
        default_options._task_name = opts.task_name
        default_options._broadcast_status = opts.broadcast_status
        return default_options

