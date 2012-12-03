from utils.watchfuncs import *

barracuda_processes = [
    Process('pipeline_manager',    ['AI_pipeline_manager.py'], node_pid('ai_pl_manager'), death = restart()),
    Process('auv_control', ['AI_control_manager.py'], node_pid('ai_auv_control'), death = restart()),
    Process('detector_control', ['AI_detection_process.py'], node_pid('ai_detector_control'), death = restart()),
    Process('location', ['AI_location.py'], node_pid('ai_location')),
    Process('task_manager', ['AI_task_manager.py'], node_pid('ai_task_manager'), death = restart(), prereq = depends_on('pipeline_manager', 'auv_control', 'detector_control', 'location')),
]

def get_processes(args):
    return barracuda_processes
