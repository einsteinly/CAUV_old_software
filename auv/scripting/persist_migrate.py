#!/usr/bin/env python2.7

import cauv.messaging as msg

import shelve
import argparse
import persist
import utils.jsonshelf

def old_dictToMessage(message_name, attr_dict):
    return getattr(msg, message_name+'Message')(**attr_dict)

def shelve_open(filename):
    #hack because bsddb is broken by default on Arch
    # 'depreciated module' doesn't mean you should hack it out
    # breaking perfectly good modules in the process...
    # see https://bugs.archlinux.org/task/25058
    try:
        shelf = shelve.open(filename)
    except ImportError:
        import bsddb3
        _db = bsddb3.hashopen(filename)
        shelf = shelve.Shelf(_db)
    return shelf

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-f', '--shelf-file', dest='fname',
                 default='./settings-persistence.shelf',
                 action='store', help='file name to save/load from (python shelf)')
    p.add_argument('-o', '--output-dir', help = 'Directory to output json data to')

    opts, args = p.parse_known_args()

    shelf = shelve_open(opts.fname)
    json_shelf = utils.jsonshelf.JSONShelf(opts.output_dir)
    for key in shelf:
        print("Migrating: {}".format(key))
        name = key
        try:
            name = key.split(':')[0]
        except IndexError:
            pass
        message = old_dictToMessage(name, shelf[key])
        json_shelf[key.replace(':','_')] = persist.dictFromMessage(message)
