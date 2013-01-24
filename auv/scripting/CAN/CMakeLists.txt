# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

python_exe(bat_switch.py)
python_exe(power_switch.py)
python_exe(can_msg.py)
install(FILES frames.json DESTINATION config/)
