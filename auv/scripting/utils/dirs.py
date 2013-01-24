#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import os
import os.path

def config_dir(subdir):
    try:
        return os.path.join(os.environ["VIRTUAL_ENV"], "config", subdir)
    except KeyError:
        return subdir
