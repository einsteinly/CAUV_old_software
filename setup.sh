#! /bin/bash

set -e

if [ -a ~/.hgrc ]; then
    echo "not creating ~/.hgrc (already exists)"
else
    echo "creating ~/.hgrc"
    # add user name and user id strings
    whoami | xargs getent passwd | sed "s/\([^:]\+\):[x:0 -9]*:\([^:,]*\).*/[ui]\nusername = \2 <\1@cam.ac.uk>\n/" >> ~/.hgrc
fi

if grep -q "^[[:space:]]*umask[[:space:]]\+002" ~/.profile; then
    echo "umask already set correctly in ~/.profile"
elif grep -q "^[[:space:]]*umask[[:space:]]\+022" ~/.profile; then
    echo "changing umask setting in ~/.profile"
    sed -i.cauv-saved "s/^\([[:space:]]*\)umask[[:space:]]\+022/\1umask 002/" ~/.profile
else
    echo "adding umask setting to ~/.profile"
    echo "umask 002" >> ~/.profile
fi

if grep -q "^alias cmake='/societies/cauv/install/bin/cmake'" ~/.bashrc; then
    echo "not adding cmake alias to ~/.bashrc (already exists)"
else
    echo "adding cmake alias to ~/.bashrc"
    echo "alias cmake='/societies/cauv/install/bin/cmake'" >> ~/.bashrc
fi

if grep -q "^alias bjam='/societies/cauv/install/bin/bjam'" ~/.bashrc; then
    echo "not adding bjam alias to ~/.bashrc (already exists)"
else
    echo "adding bjam alias to ~/.bashrc"
    echo "alias bjam='/societies/cauv/install/bin/bjam'" >> ~/.bashrc
fi


