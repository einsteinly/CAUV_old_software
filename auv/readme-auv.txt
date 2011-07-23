The auv directory contains code that runs on the submarine during normal
operation (and a few(!) test and development toys for some of that code).

In order of importance and relevance:

scripting/   - python scripts to control the high-level actions of the AUV
control/     - low-level control program: this program runs PID controllers and
               communicates in real-time to the Main Control Board
img-pipeline/  - image processing pipeline code, see img-pipeline/nodes/ for
                 how to add a new type of image processing operation
python-module/ - python module used by scripting: contains the c++
                 (boost-python) for the python access to the messaging system.
sonar/       - sonar library and standalone application for testing
pinger/      - code for hydrophone communication / processing (unknown state?)
pervert/     - debug viewing library (incomplete)


module/      - library used by control to talk to the Main Control Board by USB
lib/         - various 3rd-party libraries (xsens stuff and messaging stuff)


fileinput/   - tool for sending images from files over the messaging system:
               this is not used as img-pipeline can read directly from files
               (including video files)
embedded/    - some embedded code for battery management (not currently used)
