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



===========================================================
== Getting Started on Windows (Still a work in progress) ==

Prerequisites:

tortoisehg:
http://tortoisehg.bitbucket.org/download/index.html

filezilla / some other SCP program:
http://filezilla-project.org/download.php

Vim / Your Favourite Text Editor

CMake: 2.8.2
http://www.cmake.org/cmake/resources/software.html

Qt 4.5.?? SDK 
(trying 4.7.0, libs visual studio version!):
http://qt.nokia.com/downloads
http://qt.nokia.com/downloads/windows-cpp-vs2008

Visual Studio
(Tested with Visual c++ 2010 express):
http://www.microsoft.com/express/downloads/

7-zip (to unzip .tar.gz file for FTGL)

Freetype: 2.4.3: vc2008 files converted to 2010, build all debug/release
multi/single variants
http://download.savannah.gnu.org/releases/freetype/
(remember to set the FREETYPE environment variable to freetype sources so that
$FREETYPE/include is where the headers are)

FTGL: (2.1.3 rc5 tested), built with vc8 files, ftgl_dll, converted to 2010,
expects 2.3.4 freetype lib -- have to edit the linked libraries config to
change it to the correct 2.4.3 freetype version, and add the correct
objs/win32/vc2008 lib subdirectory to the search path
http://sourceforge.net/projects/ftgl/

boost 1.43.0:
http://www.boost.org/users/news/version_1_43_0
(remember to set BOOST_ROOT)

OpenCV: 2.1 Windows, full install, add to PATH
http://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.1/

Spread: 4.1 (convert project file and build all libspread and the spread
program using vs2010)
http://www.spread.org/download.html

libssrcspread: 1.06
Copy the files from the SRCF or someone else... extracting an xz file on
windows is more trouble than it's worth

Pthreads: pthreads-w32-2-8-0-release.exe
http://ftp.gwdg.de/pub/linux/sources.redhat.com/pthreads-win32/pthreads-w32-2-8-0-release.exe

Don't Use Python 2.7: DOES NOT WORK WITH CHEETAH:
http://www.python.org/download/releases/2.7/

Python 2.6 (64-bit version doesn't seem to play with setuptools -- use 32-bit
version)
http://www.python.org/download/releases/2.6/

Python Lex Yacc:
Python Cheetah:
Use setuptools for windows to install them
(http://pypi.python.org/pypi/setuptools#files)

Python Cheetah:
Doesn't install with easy_install... install from source with
python setup.py install
http://pypi.python.org/pypi/Cheetah/2.4.3

FTDI and librt: don't bother -- only needed for control: set them to something
random in CMake

Install XXD: 
It's included with Vim, so install that :)
Add the Vim directory to $PATH, e.g.
C:\Program Files (x86)\Vim\vim73
http://www.vim.org/download.php



