#!/usr/bin/env python2.7

import threading
import Queue
import argparse
import collections
import time
import heapq
import traceback
import functools
import inspect
from cauv.debug import debug, warning, error, info

class Heap(object):
    """Thin wrapper around some heapq functions"""
    def __init__(self):
        self.__heap = []

    def push(self, item):
        heapq.heappush(self.__heap, item)

    def pop(self):
        if len(self.__heap) == 0:
            raise Queue.Empty()
        return heapq.heappop(self.__heap)

    def peek(self):
        if len(self.__heap) == 0:
            raise Queue.Empty()
        return self.__heap[0]

class Event(object):
    """Represents an event to be executed in an EventLoop at some time in the
    future"""
    n_events = 0
    def __init__(self, func, args, kargs, priority = 0, run_at = None):
        """func: function to be called
           args, kargs: argument and keyword arguments to the function
           priority: priority of function (lower is given priority)
           run_at: time in float seconds to run function at as reported by
                   time.time(). If None, run immediately"""
        self.func = func
        self.args = args
        self.kargs = kargs
        self.priority = priority
        self.run_at = run_at
        self.sequence = self.n_events
        #true if function is waiting to be executed at a given time
        self.waiting = False
        #n_events ensures order is preserved for equal priorities
        Event.n_events += 1

    def execute(self):
        try:
            ret = self.func(*self.args, **self.kargs)
            if ret is not None:
                warning("Warning!, returning value from event {}".format(self.func.func_name))
        except Exception:
            error("Exception in event: {}".format(traceback.format_exc().encode("ascii", "ignore")))

    def __call__(self):
        self.execute()

    def __lt__(self, other):
        try:
            if self.waiting:
                return self.run_at < other.run_at
            else:
                return (self.priority, self.n_events) < \
                       (other.priority, other.n_events)
        except (TypeError, AttributeError):
            return NotImplemented

    def __gt__(self, other):
        return not self.__lt__(other)

class EventError(Exception):
    pass

class EventOptions(object):
    def __init__(self, other = None):
        self.delay = None
        self.priority = 0

#decorators which turn functions into events. Calling puts the original function
#onto the event queue
#A whole lot of this is trying to uphold the principle of least surprise, so the
#resulting object still acts like a function for the purposes of other
#decorators. Unfortunatly it does mean the way the internals work is quite
#surprising
class event_func(object):
    """A decorator which turns the decorated function into an event. Calling the
    function puts the original function onto the event queue. If the wrapped
    function is a method of an EventLoop class, then the loop is automagically
    extracted Otherwise, it must be assigned explicitely""" 
    def __init__(self, func):
        """func: function to wrap. Can also be an event_func, in which case
        loop, options, and wrapped func are copied"""
        try:
            self.options = func.options
            self.func = func.func
            self.loop = func.loop
            self.__dict__.update(func.__dict__)
        except AttributeError:
            self.__dict__.update(func.__dict__)
            self.options = EventOptions()
            self.func = func
            self.func_code = func.func_code
            self.__sync_loop()
        functools.update_wrapper(self, func)

    def __sync_loop(self):
        try:
            self.loop = self.func.__self__.event_loop
        except AttributeError:
            self.loop = None

    #This means methods can be decorated as well as self extracted, while still
    #working with functions and even bound methods.
    def __get__(self, obj, cls):
        r = self.__class__(self)
        r.func = self.func.__get__(obj, cls)
        r.__sync_loop()
        return r
    
    def add_event(self, func, args, kargs):
        """Add func to assigned event loop and options with given arguments"""
        if self.loop is None:
            raise EventError("Tried to trigger event {} without an event loop".format(self))
        self.loop.add_event(Event(func, args, kargs,
            run_at = (time.time() + self.options.delay) if self.options.delay is not None else None,
            priority = self.options.priority))

    def trigger(self, args = (), kargs = {}, delay = None, priority = 0):
        """Trigger the event with arguments, delay, and priority given"""
        self.loop.add_event(Event(self.func, args, kargs,
            run_at = (time.time() + delay) if delay is not None else None,
            priority = priority))

    def __call__(self, *args, **kargs):
        self.add_event(self.func, args, kargs)

    def __repr__(self):
        return "{} wrapping {} (delay = {}, priority = {}, loop = {})".format(
                self.__class__.__name__, self.func, self.options.delay, self.options.priority,
                self.loop)

AutoStartEvent = object()

def repeat_event(time_period = 1, autostart = False):
    """Decorator which represents an event which should repeat indefinitely with
    given time period. If autostart is true and the decorated function is a
    method of an EventLoop class, it is automagically trigerred when the
    EventLoop is initialised"""
    class repeat_event_func(event_func):
        def __init__(self, *args, **kargs):
            super(repeat_event_func, self).__init__(*args, **kargs)
            if self.options.delay is None:
                self.options.delay = time_period

        def call_wrap(self, args, kargs):
            t = time.time()
            self.add_event(self.call_wrap, (args, kargs), {})
            self.func(*args, **kargs)
            if self.options.delay is not None and \
               t + self.options.delay < time.time():
                warning(("Warning: repeating function {} taking longer to " + 
                          "execute than time period!").format(self))

        def __call__(self, *args, **kargs):
            self.add_event(self.call_wrap, (args, kargs), {})

    if autostart:
        repeat_event_func.autostart = AutoStartEvent
    return repeat_event_func

def event_priority(priority):
    """A decorator which sets the priority of an event_func"""
    def set_priority(func):
        func.options.priority = priority
        return func
    return set_priority

class EventLoop(object):
    def __init__(self, size = 0):
        super(EventLoop, self).__init__()
        self.event_queue = Queue.PriorityQueue(size)
        self.waiting_queue = Heap()
        self.running = True
        self.event_loop = self
        for name, member in inspect.getmembers(self):
            try:
                if member.autostart is AutoStartEvent:
                    member()
            except AttributeError:
                pass

    def run(self):
        while self.running:
            timeout = 1
            try:
                while self.waiting_queue.peek().run_at <= time.time():
                    self.waiting_queue.pop().execute()
                    self.event_queue.task_done()
                timeout = self.waiting_queue.peek().run_at - time.time()
            except Queue.Empty:
                pass
            try:
                event = self.event_queue.get(timeout=timeout)
                if event.run_at is not None:
                    current_time = time.time()
                    if event.run_at > current_time:
                        event.waiting = True
                        self.waiting_queue.push(event)
                        continue
                event.execute()
                self.event_queue.task_done()
            except Queue.Empty:
                pass

    def start(self):
        self.thread = threading.Thread(target = self.run)
        self.thread.start()

    def add_event(self, event):
        self.event_queue.put(event)

    def stop(self):
        self.running = False

class TestEvent(EventLoop):
    def __init__(self):
        EventLoop.__init__(self)

    def echo(self, arg):
        print(arg)

    @event_priority(1)
    @event_func
    def echo2(self, arg):
        print(arg)

    @repeat_event(0.5, True)
    def echo3(self):
        print('repeating')

    @repeat_event(0.1, False)
    def bad_function(self):
        time.sleep(0.3)


if __name__ == "__main__":
    t = TestEvent()
    try:
        t.add_event(Event(t.echo, ('testing',), {}))
        t.add_event(Event(t.echo, ('testing_wait',), {}, run_at = time.time() + 3))
        t.echo2.trigger(args = ('testing_trigger',), delay = 3)
        t.echo2('testing2')
        t.start()
        time.sleep(4)
    finally:
        t.stop()
