#!/bin/sh
SRCFILE="$0"
SRCFILE=`readlink -f "$SRCFILE"`
SRCDIR=`dirname $SRCFILE`
SRCDIR=`dirname $SRCDIR`
echo $SRCDIR
cd "$SRCDIR"
source bin/activate_reloc
pip install cauv.pybundle
virtualenv --relocatable $VIRTUAL_ENV
rm /cauv/builds/current
ln -s /cauv/builds/current $VIRTUAL_ENV
