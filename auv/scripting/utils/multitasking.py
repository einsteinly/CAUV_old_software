# CAUV utility module: multi-tasking classes and functions

def spawnDaemon(func):
    'Do the Unix double-fork magic to spawn a daemon.'
    import sys, os
    # see http://code.activestate.com/recipes/66012-fork-a-daemon-process-on-unix/
    # do the UNIX double-fork magic, see Stevens' "Advanced
    # Programming in the UNIX Environment" for details (ISBN 0201563177)
    try:
        pid = os.fork()
        if pid > 0:
            # parent process
            return
    except OSError, e:
        print >>sys.stderr, "fork #1 failed: %d (%s)" % (e.errno, e.strerror)
        os._exit(os.EX_OSERR)

    # decouple from parent environment
    # ... don't (bad daemon ettiquite, but it's necessary for now)
    #os.chdir("/")
    os.setsid()
    #os.umask(0)

    # do second fork
    # os._exit() is less 'nice' because it doesn't allow python to fully unwind
    # like a normal exit (executing finallys and such), but in this case we don't
    # want it to because the forked environment is probably bogus since threads
    # aren't carried across the fork and other such problems. This also means
    # that func() really should only be doing very simple stuff like spawning a
    # new process or otherwise used with care.
    try:
        pid = os.fork()
        if pid > 0:
            # exit from second parent
            os._exit(os.EX_OK)
    except OSError, e:
        print >>sys.stderr, "fork #2 failed: %d (%s)" % (e.errno, e.strerror)
        os._exit(os.EX_OSERR)
    # do stuff
    func()

    # all done
    os._exit(os.EX_OK)
