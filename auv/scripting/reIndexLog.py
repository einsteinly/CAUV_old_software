#!/usr/bin/env python2.7

# Standard Library Modules
import argparse

# CAUV Modules 
from utils import CHIL

def reIndex(dirname):
    reindexer = CHIL.ReIndexer(dirname)
    reindexer.reIndex()

if __name__ == '__main__':
    p = argparse.ArgumentParser(description='re-generate the index files of a .chil log')
    p.add_argument('file', metavar='FILE', type=str)
    opts = p.parse_args()

    reIndex(opts.file)

