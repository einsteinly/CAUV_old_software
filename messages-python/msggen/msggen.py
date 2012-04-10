#!/usr/bin/env python2.7

from __future__ import with_statement

import os
import sys
import hashlib
import collections
from optparse import OptionParser

from msggenyacc import parser
from Cheetah.Template import Template

import gencpp
import genc
import genpython
import genchil

def writeIfChanged(filename, text, options):
    if not options.all and os.path.exists(filename):
        texthash = hashlib.md5()
        texthash.update(text)
        with open(filename, "r") as file:
            filehash = hashlib.md5()
            filehash.update(file.read())
        if filehash.digest() == texthash.digest():
            #print "File %s not changed, not writing" % filename
            return []

    if not options.nowrite:
        print "Writing file %s" % filename
        with open(filename, "w") as file:
            file.write(text)
    return [filename]

def main():
    p = OptionParser(usage="usage: %prog [options] INPUT")
    p.add_option("-l", "--lang",
                 choices=["c++-cpp-files", "c++-files", "c++", "c", "java", "python", "chil"],
                 default="c++",
                 metavar="LANG",
                 help="output language (java, python, c++, or c) [default: %default]")
    p.add_option("-a", "--all",
                 action="store_true",
                 dest="all",
                 default=False,
                 help="if set, will write over all files. Otherwise, only writes over if md5 hases differ")
    p.add_option("-n", "--nowrite",
                 action="store_true",
                 dest="nowrite",
                 default=False,
                 help="if set, will just output the filenames that would have been written")
    p.add_option("-o", "--output",
                 type="string",
                 metavar="FILE",
                 help="output filename(s) prefix (file extension will be added depending on language) [default: INPUT]")
    p.add_option("-t", "--template-dir", default=os.path.join(os.path.dirname(sys.argv[0]), 'templates'),
                 dest='template_dir', metavar="TEMPLATE_DIR",
                 help="look for template files here [default: %default]")
    p.add_option("-p", "--package",
                 type="string",
                 default="cauv",
                 metavar="PACKAGE",
                 help="package to put java files in, ignored for other languages [default: %default]")

    options, args = p.parse_args()
    
    if len(args) < 1:
        p.error("no input file specified")
    elif len(args) > 1:
        p.error("only one input file allowed")

    if options.output == None:
        options.output = args[0]
    output_dir = os.path.abspath(options.output)

    with open(args[0], "r") as file:
        data = file.read()

    tree = parser.parse(data)
    
    msgdir = options.template_dir

    filesWritten = []

    if options.lang == "c++":
        # -----------------
        #  C++
        # -----------------
        msgdir = os.path.join(msgdir, 'cpp')
        
        output_types_dir = os.path.join(output_dir,"types")
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        if not os.path.exists(output_types_dir):
            os.makedirs(output_types_dir)

        output_files = gencpp.get_output_files(tree)
    
    elif options.lang == "c":
        # -----------------
        #  C
        # -----------------
        msgdir = os.path.join(msgdir, 'c')

        output_files = genc.get_output_files(tree)

    elif options.lang == "python":
        # -----------------
        #  Python
        # -----------------
        msgdir = os.path.join(msgdir, 'python')
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        output_files = genpython.get_output_files(tree)

    elif options.lang == 'chil':
        # -----------------
        #  CHIL Decode
        # -----------------
        output_files = genchil.get_output_files(tree)

    for output in output_files:
        t = Template(file = os.path.join(msgdir, output.template_file), searchList=output.search_list)
        filesWritten += writeIfChanged(os.path.join(output_dir, output.output_file), str(t), options)
        
    if options.nowrite:
        print ";".join((os.path.abspath(f) for f in filesWritten))
    return 0

if __name__ == '__main__':
    ret = main()
    sys.exit(ret)
