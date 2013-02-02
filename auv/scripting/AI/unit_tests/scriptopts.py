#!/usr/bin/env python2.7

import unittest
import AI
import AI.options

class SimpleTestScript(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.test_int = 1
            self.test_float = 1.0
            self.test_str = "abc"
            self.test_bool = True
            self.exit_success = True

    def run(self):
        return self.exit_success

class NestedOptions(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c
    def to_dict(self):
        return {"a" : self.a,
                "b" : self.b,
                "c" : self.c}

    def from_dict(self, opts_dict):
        self.a = opts_dict["a"]
        self.b = opts_dict["b"]
        self.c = opts_dict["c"]

class NestedTestScript(AI.Script):
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.nest_a = NestedOptions(1,2,3)
            self.nest_b = NestedOptions("a", "b", "c")
            self.nonnested = "test"

class TestUtilFunc(unittest.TestCase):
    def test_flatten_trivial(self):
        simple_dict = {"1" : 1, "2": 2, "3" : 3}
        flat_simple_dict = AI.options.flatten_dict(simple_dict)
        self.assertEqual(simple_dict, flat_simple_dict)

    def test_flatten_simple(self):
        nested_dict = {"1": {"1" : 1, "2" : 2}, "2" : 2, "3" : 3}
        result = {"1/1" : 1, "1/2" : 2, "2" : 2, "3" : 3}
        self.assertEqual(AI.options.flatten_dict(nested_dict), result)

class TestScriptOptions(unittest.TestCase):
    def test_default_options(self):
        options = SimpleTestScript.DefaultOptions()
        expected_result = {"test_int" : 1,
                           "test_float" : 1.0,
                           "test_str" : "abc",
                           "test_bool" : True,
                           "exit_success" : True}
        self.assertEqual(options.to_dict(), expected_result)

    def test_get_options(self):
        options = SimpleTestScript.get_options()
        self.assertEqual(options.test_int, 1)
        self.assertEqual(options.test_float, 1.0)
        self.assertEqual(options.test_str, "abc")
        self.assertEqual(options.test_bool, True)

    def test_options_passed(self):
        SimpleTestScript._test_args = ["--test_int", "2",
                                       "--test_float", "4.3",
                                       "--test_str", "def",
                                       "--test_bool", "False"]
        options = SimpleTestScript.get_options()
        self.assertEqual(options.test_int, 2)
        self.assertEqual(options.test_float, 4.3)
        self.assertEqual(options.test_str, "def")
        self.assertEqual(options.test_bool, False)
        del SimpleTestScript._test_args

    def test_nested_options(self):
        options = NestedTestScript.get_options()
        self.assertEqual(options.nest_a.a, 1)
        self.assertEqual(options.nest_a.b, 2)
        self.assertEqual(options.nest_a.c, 3)
        self.assertEqual(options.nest_b.a, "a")
        self.assertEqual(options.nest_b.b, "b")
        self.assertEqual(options.nest_b.c, "c")
        self.assertEqual(options.nonnested, "test")

    def test_nested_options_passed(self):
        NestedTestScript._test_args = ["--nest_a/a", "4",
                                       "--nest_b/c", "6"]
        options = NestedTestScript.get_options()
        self.assertEqual(options.nest_a.a, 4)
        self.assertEqual(options.nest_a.b, 2)
        self.assertEqual(options.nest_a.c, 3)
        self.assertEqual(options.nest_b.a, "a")
        self.assertEqual(options.nest_b.b, "b")
        self.assertEqual(options.nest_b.c, "6")
        del NestedTestScript._test_args

    def test_cmd_opts(self):
        options = NestedTestScript.DefaultOptions()
        options.nest_a.a = 66
        opts = options.to_cmd_opts()
        self.assertEqual(opts.index("66") - opts.index("--nest_a/a"), 1)
        NestedTestScript._test_args = opts
        options = NestedTestScript.get_options()
        self.assertEqual(options.nest_a.a, 66)
        del NestedTestScript._test_args

if __name__ == "__main__":
    unittest.main()
