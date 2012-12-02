import os
import os.path

def config_dir(subdir):
    try:
        return os.path.join(os.environ["VIRTUAL_ENV"], "config", subdir)
    except KeyError:
        return subdir
