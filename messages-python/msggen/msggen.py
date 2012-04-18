#!/usr/bin/env python2.7

from __future__ import with_statement

import os
import sys
import fnmatch
from optparse import OptionParser

p = OptionParser(usage="usage: %prog [options] INPUT")
p.add_option("-l", "--lang",
             choices=["c++", "c", "java", "python", "chil"],
             default="c++",
             metavar="LANG",
             help="Output language (java, python, c++, or c) [default: %default]")
p.add_option("-a", "--all",
             action="store_true",
             dest="all",
             default=False,
             help="If set, will write over all files. Otherwise, only writes over if the template files if the md5 sums differ")
p.add_option("-n", "--nowrite",
             action="store_true",
             dest="nowrite",
             default=False,
             help="If set, will just output the filenames that would have been written")
p.add_option("-o", "--output",
             type="string",
             metavar="FILE",
             help="Output filename(s) prefix (file extension will be added depending on language) [default: INPUT]")
p.add_option("-t", "--template-dir", default=os.path.join(os.path.dirname(sys.argv[0]), 'templates'),
             dest='template_dir', metavar="TEMPLATE_DIR",
             help="Look for template files here [default: %default]")
p.add_option("-p", "--package",
             type="string",
             default="cauv",
             metavar="PACKAGE",
             help="Package to put java files in, ignored for other languages [default: %default]")
p.add_option("-c", "--cmake-out",
             type="string",
             help="Cmake file to write out containing generated files (only written when files would be changed)")
p.add_option("-v", "--cmake-prefix",
             type="string",
             help="Prefix for cmake var set in output cmake file")
p.add_option("-m", "--marker-file",
             type="string",
             help="File which represents last time msggen generated templates")

options, args = p.parse_args()

if len(args) < 1:
    p.error("no input file specified")
elif len(args) > 1:
    p.error("only one input file allowed")

messages_file = os.path.abspath(args[0])

#build speedup. This essentially tracks its own dependencies since it gets run
#every build anyway
if options.marker_file and os.path.exists(options.marker_file):
    dep_files = [messages_file] 
    for root, dirnames, filenames in os.walk(options.template_dir):
        for filename in fnmatch.filter(filenames, "*.template*"):
            dep_files.append(os.path.join(root,filename))
    for root, dirnames, filenames in os.walk(os.path.dirname(sys.argv[0])):
        for filename in fnmatch.filter(filenames, "*.py"):
            dep_files.append(os.path.join(root,filename))
    marker_mtime = os.path.getmtime(options.marker_file)
    for dep_file in dep_files:
        if marker_mtime < os.path.getmtime(dep_file):
            break
    else:
        exit()

import hashlib

from msggenyacc import parser
from Cheetah.Template import Template

import gencpp
import genc
import genpython
import genchil

if options.output == None:
    options.output = args[0]
output_dir = os.path.abspath(options.output)

with open(messages_file, "r") as msg_file:
    data = msg_file.read()

#bump when serialisation format changes
serialise_format_ver = 1

tree = parser.parse(data)

#print(tree)
tree.build_lookup()
for group in tree['groups']:
    for message in group.messages:
        h = hashlib.md5()
        h.update(str(serialise_format_ver))
        message.add_to_hash(h)
        message.check_hash = int(h.hexdigest()[0:8],16)
        #print("{:>30} : 0x{:0>8x}".format(message.name, message.check_hash))

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

def writeIfChanged(output_file, template):
    text = str(template)
    if os.path.exists(output_file) and not options.all:
        texthash = hashlib.md5()
        texthash.update(text)
        with open(output_file, "r") as file:
            filehash = hashlib.md5()
            filehash.update(file.read())
        if filehash.digest() == texthash.digest() and not options.all:
            #print "File {} not changed, not writing".format(output_file)
            return []

    if not options.nowrite:
        with open(output_file, "w") as file:
            print("Writing file {}".format(output_file))
            file.write(text)
    return [output_file]

cmake_files = []

for output in output_files:
    template_file = os.path.join(msgdir, output.template_file)
    t = Template(file = template_file, searchList=output.search_list)
    output_file = os.path.join(output_dir, output.output_file)
    cmake_files.append(output_file)
    filesWritten += writeIfChanged(output_file, t)

if not options.nowrite and options.marker_file:
    with open(options.marker_file, "w") as marker_file:
        marker_file.write("")
    
if options.nowrite:
    print ";".join((os.path.abspath(f) for f in filesWritten))

if options.cmake_out:
    if filesWritten or not os.path.exists(options.cmake_out):
        with open(options.cmake_out, "w") as cmake_file:
            print("writing cmake file {}".format(options.cmake_out))
            files = "\n".join((os.path.abspath(f) for f in cmake_files))
            cmake_file.write("SET({}_FILES {})\n".format(options.cmake_prefix, files))
