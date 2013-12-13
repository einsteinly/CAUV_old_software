This top-level directory contains, in order of importance:

Basic compilation:
==================

READ THE REST OF THIS DOCUMENT (especially installing 
$cd <somewhere_not_in_the_repo>
$mkdir build
$cd build
$cmake <path_to_software.hg>
$make -j<however many cores you have + 1>
if you run out of memory compiling (even if it just starts swapping to disk), reduce the -jnumber passed to make!

Most programs will be placed in build/bin. Most interesting ones are gui,
scratch, and img-pipeline.

Important things:
=================

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

Less important things:
=======================

cmake/            - files for the build system
CMakeLists.txt    - top-level build file for the CMake build system


Installing Dependencies
=======================

Check the wiki (https://data.cambridgeauv.co.uk/wiki/InstallInstructions,
                same username/password as for access to repository) for up to date instructions.
Installing on Ubuntu with the already set up packages will be a lot easier than
compiling manually!

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

If you've installed all these, and something still seems to be missing, please
add it to this list! 
