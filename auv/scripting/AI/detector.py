from cauv.debug import debug, warning, error, info
import proc
import cauv.node as node

class Detector(proc.Proc):
    def __init__(self):
        proc.Proc.__init__(self)
        self.options = self.get_options()
        self.node = node.Node(self.__class__.__name__ + "Script")
        self.node.addObserver(self)

    def fire(self, timeout):
        if timeout == 0:
            info("Detector stopped firing")
        else:
            info("Detector fired with timout of {} seconds".format(timeout))

    @classmethod
    def entry(cls):
        cls.get_options()
        instance = cls()

        ret = False
        try:
            ret = instance.run()
        finally:
            instance.unload_pipeline("")
            info("Detector pipelines cleaned up")
