#! /bin/bash

set -e

if [ -a ~/.hgrc ] ; then
    echo "not creating ~/.hgrc (already exists)"
else
    echo "creating ~/.hgrc"
    # add user name and user id strings
    whoami | xargs getent passwd | sed "s/\([^:]\+\):[x:0 -9]*:\([^:,]*\).*/[ui]\nusername = \2 <\1@cam.ac.uk>\n/" >> ~/.hgrc
fi


if grep -q "^[[:space:]]*umask[[:space:]]\+002" ${HOME}/.profile ; then
    echo "umask already set correctly in ~/.profile"
elif grep -q "^[[:space:]]*umask[[:space:]]\+022" ${HOME}/.profile ; then
    echo "changing umask setting in ~/.profile"
    sed -i.cauv-saved "s/^\([[:space:]]*\)umask[[:space:]]\+022/\1umask 002/" ${HOME}/.profile
else
    echo "adding umask setting to ~/.profile"
    echo "umask 002" >> ${HOME}/.profile
fi


CAUV_PROFILE=/societies/cauv/cauv_profile
SOURCE_CAUV_PROFILE="[ -f ${CAUV_PROFILE} ] && source ${CAUV_PROFILE}"
if grep -Fqx "${SOURCE_CAUV_PROFILE}" ${HOME}/.profile ; then
    echo "already sourcing cauv_profile"
else
    echo "sourcing cauv_profile in ~/.profile..."
    echo "${SOURCE_CAUV_PROFILE}" >> ${HOME}/.profile
fi
    



# This shouldn't be necessary after a while

function clean_up
{
    if grep -q "$1" $2 ; then
        echo "cleaning up \"$1\" from $2"
        grep -Fv "$1" $2 >$2
    fi
}

clean_up 'export PATH=/societies/cauv/install/bin:$PATH' ${HOME}/.profile
clean_up 'export PYTHONPATH=/societies/cauv/install/lib/python2.6/site-packages/:$PYTHONPATH' ${HOME}/.profile
clean_up 'export BOOST_BUILD_PATH=/societies/cauv/build/boost/' ${HOME}/.profile
clean_up 'export BOOST_ROOT=/societies/cauv/build/boost/' ${HOME}/.profile
clean_up 'export CAUV_PREFIX=/societies/cauv/install' ${HOME}/.profile
clean_up "alias cmake='/societies/cauv/install/bin/cmake'" ${HOME}/.bashrc
