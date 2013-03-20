#!/usr/bin/env python2.7
import unittest
import importlib
import imp
import traceback
from glob import glob
        
def generate_detector_case(mod_name):
    class TestDetector(unittest.TestCase):
        def test_isdetector(self):
            module = importlib.import_module('detector_library.'+mod_name)
            self.assertTrue(hasattr(module,"Detector"))
    TestDetector.__name__ = 'Test{}Detector'.format(mod_name)
    globals()['Test{}Detector'.format(mod_name)] = TestDetector
        
if __name__=='__main__':
    lib_path = imp.find_module('detector_library')[1]
    mod_names = [f[len(lib_path)+1:-3] for f in glob(lib_path+'/*.py') if f[-11:-3]!='__init__']
    for mod_name in mod_names:
        generate_detector_case(mod_name)
    unittest.main()