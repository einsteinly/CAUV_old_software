#!/usr/bin/env python2.7
import utils.event as event
import time
import unittest

class TestEventLoop(event.EventLoop):
    def __init__(self):
        event.EventLoop.__init__(self)
        self._events = []

    def echo(self, arg):
        self._events.append(arg)

    @event.event_priority(1)
    @event.event_func
    def echo2(self, arg):
        self._events.append(arg)

    @event.repeat_event(0.05, True)
    def echo3(self):
        self._events.append('repeating')

class TestEvent(unittest.TestCase):
    def test_events(self):
        t = TestEventLoop()
        expected = ['testing',
                    'testing2',
                    'repeating',
                    'repeating',
                    'repeating',
                    'repeating',
                    'repeating',
                    'testing_wait',
                    'testing_trigger',
                    'repeating',
                    'repeating']
        try:
            t.add_event(event.Event(t.echo, ('testing',), {}))
            t.add_event(event.Event(t.echo, ('testing_wait',), {}, run_at = time.time() + 0.3))
            t.echo2.trigger(args = ('testing_trigger',), delay = 0.3)
            t.echo2('testing2')
            t.start()
            time.sleep(0.4)
        finally:
            t.stop()
        self.assertEqual(expected, t._events)

if __name__ == "__main__":
    unittest.main()
