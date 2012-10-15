This top-level directory contains, in order of importance:

Important things:
=================

setup.sh        - script to setup various things for development
auv/            - folder containing code that runs on the AUV
common/         - library used by c++ programs to talk to the messaging system:
                  provides some other common utilities
messages-python/ - python code to automatically generate code for the messaging
                   system (automatically invoked by the build system)
gui/             - GUI code: c++ (Qt) GUI for configuring the image processing
                  pipeline and c++ (Qt) Graphical User Interface for
                  remote-controlling the AUV, viewing telemetry, etc.
generated/      - generated headers that are shared between multiple targets
                  (mostly messaging system headers). add messages-shared or
                  logo-shared as explicit dependencies if a target needs these
                  files.
debug/          - debug output library
utility/        - c++ files shared between gui/ and auv/ and anywhere else they
                  are needed

sim/            - Simulation node, the aim is to provide indistinguishable
                  output from the real hardware in order to test the rest of the 
                  software in a virtual environment.

No longer used:                  
(pitzdir/        - directory containing the files of the issue tracker (pitz):
                  to see a list of tasks from the command line, run:
                    pitz-tasks --detailed-view

django/         - code for the web interface of the issue tracker (pitz))


Less important things:
=======================

cmake/            - files for the build system
CMakeLists.txt    - top-level build file for the CMake build system
msg-format.txt    - description of the message format




Installing Dependencies
=======================

Check the wiki (data.cambridgeauv.co.uk/wiki, same username/password as for access to repository) for up to date instructions

Dependancies:
This is an (incomplete!) list of the libraries required for things to build
correctly:

Source Control:
    mercurial (hg), as recent as possible version

Build System:
    cmake > 2.8
    python 2.7 with Cheetah, pylexyacc

common:
    boost- >= 1.44 (but go for the most recent available), including boost-python
    python 2.7
    OpenCV (>= 2.2)
    libxs (compiled with libzmq compatibility, and pgm)
    pyzmq (compiled against libxs)

auv:
    librt (this is include in libc6 most of the time)
    libftdi
    libsensors
    PCL(http://pointclouds.org/) which requires:
        octave,
        eigen3,
        flann,
        cminpack,
        qhull
    clipper (http://www.angusj.com/delphi/clipper.php)
    gpsd
    watch.py requires:
        psutils (https://code.google.com/p/psutil/)
        psi (http://www.psychofx.com/psi/)

gui:
    Qt 4, ?with OpenGL support
    libFTGL
    libgraphviz

This is an OUTDATED list, kept only for reference only:

PACKAGES

sudo apt-get install libboost-dev cmake cmake-curses-gui python-cheetah \
python-ply python-numpy python-psutil python-setuptools qt4-dev-tools \
libqt4-dev qt4-designer qt4-doc libftdi-dev libftgl-dev libqhull-dev vim

sudo easy_install psi blist Quaternion

BUILT FROM SOURCE

These also need to be compiled from source: (email the mailing list if you
have trouble or need help)

For new ubunty install!!
Follow the steps on this website to install ffm-peg first
http://ubuntuforums.org/showthread.php?t=786095

OpenCV >= 2.3.1 (http://opencv.willowgarage.com/wiki/InstallGuide)
PCL             (http://pointclouds.org)

crossroads (with pgm support and zmq compatibility)

If you've installed all these, and something still seems to be missing, please
add it to this list! 