from AI_classes import detectorCondition, aiTask

#a dict of tasks that can be added or removed as neccessary
task_list = [
            aiTask('test', 'test', 1, detectors_enabled=False, conditions = [detectorCondition('test_detector', 'test'),]),
            aiTask('pipe_follow', 'pipe_follow', 1, conditions = [detectorCondition('pipe_detector', 'pipe_detector'),]),
            aiTask('circle_buoy', 'circle_buoy', 1, conditions = [detectorCondition('buoy_detector', 'buoy_detector'),]),
            aiTask('avoid_collision', 'avoid_collision', 1, conditions = [detectorCondition('collision_detector', 'collision_detector'),]),
            #aiTask(task_name, script, priority, running_priority=priority, detectors_enabled=False, conditions=None)
            ]

#a list of tasks to look out for at the beggining
initial_tasks = [
                'circle_buoy',
                #'test',
                'pipe_follow',
                'test',
                #'avoid_collision'
                ]
#script that runs otherwise
default_script = 'spiral'
default_script_options = {'power' : 100}
