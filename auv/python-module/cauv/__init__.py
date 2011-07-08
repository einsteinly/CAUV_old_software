# CAUV python module: messaging and higher-level control
import os, sys

try:
    cauv_python_mode = os.environ['CAUV_PYTHON_MODE']
    extra_dir = cauv_python_mode.lower()
except KeyError:
    extra_dir = 'release'
finally:
    extra_paths = []
    for path in sys.path:
        extra_paths.append(os.path.join(path, extra_dir))
    sys.path.extend(extra_paths)

