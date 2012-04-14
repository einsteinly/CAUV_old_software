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

pitzdir/        - directory containing the files of the issue tracker (pitz):
                  to see a list of tasks from the command line, run:
                    pitz-tasks --detailed-view

django/         - code for the web interface of the issue tracker (pitz)

sim/            - Simulation node, the aim is to provide indistinguishable
                  output from the real hardware in order to test the rest of the 
                  software in a virtual environment.

Less important things:
=======================

cmake/            - files for the build system
CMakeLists.txt    - top-level build file for the CMake build system
msg-format.txt    - description of the message format




Installing Dependencies
=======================

On a linux system using the apt package manager, this will install most of the
dependencies (it might take a while!)

sudo apt-get install libboost-dev cmake cmake-curses-gui python-cheetah \
python-ply python-numpy python-psutil python-setuptools qt4-dev-tools \
libqt4-dev qt4-designer qt4-doc libftdi-dev libftgl-dev libqhull-dev vim


And some of the python dependencies are only available via easy install:

sudo easy_install psi blist Quaternion


These also need to be compiled from source: (email the mailing list if you
have trouble or need help)

For new ubunty install!!
Follow the steps on this website to install ffm-peg first
http://ubuntuforums.org/showthread.php?t=786095

OpenCV >= 2.3.1 (http://opencv.willowgarage.com/wiki/InstallGuide)
PCL             (http://pointclouds.org)

either:
libspread       (http://data.cambridgeauv.co.uk/files/deps/spread-src-4.1.0.tbz)
libssrcspread   (http://data.cambridgeauv.co.uk/files/deps/libssrcspread-1.0.9.tbz)
or:
libzmq (with pgm support), pass -DZEROMQ_MESSAGING=ON to cmake. availible via
apt-get isntall libzmq-dev on ubuntu

If you've installed all these, and something still seems to be missing, please
add it to this list!  




Random technical stuff:
=======================
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
    !!! These configure commands might not be necessary
    either:
    spread (./configure --with-pic CFLAGS="-enable-shared -fPIC")
    libssrcspread (./configure --with-spread=/path/to/spread/install/probably/usr/local/ --enable-debug --disable-lua-binding --disable-perl-binding --disable-python-binding --enable-shared CXXFLAGS=-fPIC LDFLAGS=-fPIC CFLAGS=-fPIC --disable-ruby-binding --with-pic)
    or:
    libxs (compiled with libzmq compatibility)
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
    gpsd
    watch.py requires:
        psutils (https://code.google.com/p/psutil/)
        psi (http://www.psychofx.com/psi/)

gui/gui-pipeline and gui/pipeline-widget:
    Qt 4.?, with OpenGL support
    libFTGL
    libgraphviz

gui/gui:
    Qt 4 (with Qt3 support library, but NOT qt3 dev packages, as this sets all of the defaults to qt3, meaning qwt6 won't build)
    Qwt 6 (Having earlier versions installed may cause a conflict when trying to make Qwt, needs to be >=rc5)
    Marble (optional) - for the map
    OIS (optional) - for gamepad controls


