from AI_classes import detectorCondition, aiTask, aiCondition, timeoutCondition

#a dict of tasks that can be added or removed as neccessary
task_list = [
            aiTask('test', 'test', 1, detectors_enabled=False, conditions = [detectorCondition('test_detector', 'test'),]),
            aiTask('follow_pipe', 'follow_pipe', 2, conditions = [detectorCondition('pipe_detector', 'pipe_detector'),]),
            aiTask('circle_buoy', 'circle_buoy', 2, conditions = [detectorCondition('buoy_detector', 'buoy_detector'),]),
            aiTask('avoid_collision', 'avoid_collision', 10, conditions = [detectorCondition('collision_detector', 'collision_detector'),]),
            aiTask('head_to_poi', 'head_to_poi', 1, detectors_enabled=True, conditions = [detectorCondition('poi_detector', 'poi_detector'),]),
            aiTask('track_wall', 'track_wall', 1, conditions = [aiCondition('start_track_wall'),]),
            aiTask('surface', 'surface', 10, conditions = [timeoutCondition('surface_timeout', 120),])
            #aiTask(task_name, script, priority, running_priority=priority, detectors_enabled=False, conditions=None, options=task_options)
            ]

#a list of tasks to look out for at the beggining
initial_tasks = [
                'circle_buoy',
                #'test',
                'follow_pipe',
                #'test',
                #'avoid_collision',
                'surface',
                ]
#script that runs otherwise
default_script = 'test'
default_script_options = {'power' : 100}
