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

if grep -q "^export PATH=/societies/cauv/install/bin:\$PATH" ~/.profile; then
    echo "not adding cauv/install/bin to PATH in ~/.profile (already set)"
else
    echo "adding cauv/install/bin to PATH in ~/.profile"
    echo "export PATH=/societies/cauv/install/bin:\$PATH" >> ~/.profile
fi

if grep -q "^export BOOST_BUILD_PATH=/societies/cauv/build/boost/" ~/.profile; then
    echo "not setting BOOST_BUILD_PATH in ~/.profile (already set)"
else
    echo "setting BOOST_BUILD_PATH in ~/.profile"
    echo "export BOOST_BUILD_PATH=/societies/cauv/build/boost/" >> ~/.profile
fi

if grep -q "^export BOOST_ROOT=/societies/cauv/build/boost/" ~/.profile; then
    echo "not setting BOOST_ROOT in ~/.profile (already set)"
else
    echo "setting BOOST_ROOT in ~/.profile"
    echo "export BOOST_ROOT=/societies/cauv/build/boost/" >> ~/.profile
fi

if grep -q "^export CAUV_PREFIX=/societies/cauv/install" ~/.profile; then
    echo "not setting CAUV_PREFIX in ~/.profile (already set)"
else
    echo "setting CAUV_PREFIX in ~/.profile"
    echo "export CAUV_PREFIX=/societies/cauv/install" >> ~/.profile
fi

