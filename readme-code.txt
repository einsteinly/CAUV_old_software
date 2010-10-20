This top-level directory contains, in order of importance:

Important things:
auv/		    - folder containing code that runs on the AUV
messages-python/ - python code to automatically generate code for the messaging
                   system (automatically invoked by the build system)
gui/		    - java (Qt Jambi) Graphical User Interface for
                  remote-controlling the AUV, viewing telemetry, etc.
gui-pipeline/ 	- c++ (Qt) GUI for configuring the image processing pipeline
generated/      - generated headers that are shared between multiple targets
                  (mostly messaging system headers). add messages-shared or
                  logo-shared as explicit dependencies if a target needs these
                  files.
utility/	    - c++ files shared between gui/ and auv/ and anywhere else they
                  are needed (headers only)

Less important things:
cmake/		    - files for the build system
CMakeLists.txt	- top-level build file for the CMake build system
embedded/	    - some embedded code for battery management (not currently used)
