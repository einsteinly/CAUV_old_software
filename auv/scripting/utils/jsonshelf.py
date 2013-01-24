#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import json
import UserDict
import os
import errno

class JSONShelf(UserDict.DictMixin):
    def __init__(self, directory):
        self.directory = directory
        try:
            os.makedirs(self.directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise

    def __getitem__(self, key):
        try:
            with open(os.path.join(self.directory, key + '.json'), 'r') as json_file:
                return json.load(json_file)
        except (IOError, ValueError):
            raise KeyError

    def __setitem__(self, key, value):
        try:
            with open(os.path.join(self.directory, key + '.json'), 'w') as json_file:
                json.dump(value, json_file, sort_keys=True, indent=4)
        except IOError:
            raise KeyError

    def __delitem__(self, key):
        try:
            os.unlink(os.path.join(self.directory, key + '.json'))
        except IOError:
            raise KeyError

    def keys(self):
        return [filename.rsplit('.',1)[0] for filename in
                os.listdir(self.directory) if filename.endswith('.json')]
