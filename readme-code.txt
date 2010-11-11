This top-level directory contains, in order of importance:

Important things:
setup.sh		- script to setup various things for development
auv/		    - folder containing code that runs on the AUV
messages-python/ - python code to automatically generate code for the messaging
                   system (automatically invoked by the build system)
gui/ 		    - GUI code: c++ (Qt) GUI for configuring the image processing
                  pipeline and a java (Qt Jambi) Graphical User Interface for
                  remote-controlling the AUV, viewing telemetry, etc.
generated/      - generated headers that are shared between multiple targets
                  (mostly messaging system headers). add messages-shared or
                  logo-shared as explicit dependencies if a target needs these
                  files.
utility/	    - c++ files shared between gui/ and auv/ and anywhere else they
                  are needed (headers only)

pitzdir/        - directory containing the files of the issue tracker (pitz):
                  to see a list of tasks from the command line, run:
                    pitz-tasks --detailed-view

django/         - code for the web interface of the issue tracker (pitz)

Less important things:
cmake/		    - files for the build system
CMakeLists.txt	- top-level build file for the CMake build system
embedded/	    - some embedded code for battery management (not currently used)


Random technical stuff:
Dependancies:
This is an (incomplete!) list of the libraries required for things to build
correctly:

Source Control:
    mercurial (hg), as recent as possible version

Build System:
    cmake > 2.8
    python 2.6 with Cheetah, pylexyacc

auv:
    librt
    libftdi
    spread (?static and shared?)
    libssrcspread (?static and shared?)
    boost-??? (default build of boost will do), boost 1.43 works
    OpenCV (2.1 works)
    bjam
    boost-python
    python 2.6


gui/gui-pipeline and gui/pipeline-widget:
    Qt 4.?, with OpenGL support
    libFTGL

gui/gui-java:
    Qt Jambi
    ???


