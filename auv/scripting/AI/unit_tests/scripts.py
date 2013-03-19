#!/usr/bin/env python2.7
import unittest
import importlib
import imp
import traceback
from glob import glob
        
def generate_script_case(mod_name):
    class TestScript(unittest.TestCase):
        def test_isscript(self):
            module = importlib.import_module('script_library.'+mod_name)
            self.assertTrue(hasattr(module,"Script"))
    TestScript.__name__ = 'Test{}Script'.format(mod_name)
    globals()['Test{}Script'.format(mod_name)] = TestScript
        
if __name__=='__main__':
    lib_path = imp.find_module('script_library')[1]
    mod_names = [f[len(lib_path)+1:-3] for f in glob(lib_path+'/*.py') if f[-11:-3]!='__init__']
    for mod_name in mod_names:
        generate_script_case(mod_name)
    unittest.main()