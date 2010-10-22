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





