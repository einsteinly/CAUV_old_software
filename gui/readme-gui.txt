This directory contains:

gui            - files for new unified Qt/C++ gui
gui-pipeline   - small c++/Qt wrapper program for the pipeline-widget GUI element
                 to run it as a standalone program
pipeline-widget - c++/Qt GUI element (actually a QWidget) containing an independent
                  GUI for controlling and the image processing pipeline: can be
                  included in Qt applications to provide this functionality.


CMakeLists.txt - build file pointing to gui, gui-pipeline and pipeline-widget:
                      the java gui is not integrated into the build system
