from AI_classes import detectorCondition, aiTask, aiCondition, timeoutCondition, timeCondition

#a dict of tasks that can be added or removed as necessary
task_list = [
            aiTask('start', 'start', 1, detectors_enabled=True),
            aiTask('test', 'test', 2, detectors_enabled=False, conditions = [detectorCondition('test_detector', 'test'),]),
            aiTask('follow_pipe', 'follow_pipe', 3, conditions = [detectorCondition('pipe_detector', 'pipe_detector'),]),
            aiTask('circle_buoy', 'circle_buoy', 3, conditions = [detectorCondition('buoy_detector', 'buoy_detector'),]),
            aiTask('avoid_collision', 'avoid_collision', 10, conditions = [detectorCondition('visual_collision_detector', 'visual_collision_detector'),]),
            aiTask('head_to_poi', 'head_to_poi', 2, detectors_enabled=True, conditions = [detectorCondition('poi_detector', 'poi_detector'),]),
            aiTask('track_wall', 'track_wall', 3, conditions = [aiCondition('start_track_wall'),]),
            aiTask('surface', 'surface', 10, conditions = [timeoutCondition('surface_timeout', 180),]),
            aiTask('follow_cam', 'follow_cam', 1, detectors_enabled=True),
            aiTask('collide', 'test', 4, conditions = [detectorCondition('sonar_collision_detector', 'sonar_collision_detector')]),
            #aiTask(task_name, script, priority, running_priority=priority, detectors_enabled=False, conditions=None, options=task_options)
            ]

#a list of tasks to look out for at the beginning
initial_tasks = [
                #'start',
                #'circle_buoy',
                #'test',
                #'follow_pipe',
                #'test',
                #'avoid_collision',
                #'surface',
                #'follow_cam'
                'collide'
                ]
#script that runs otherwise
default_script = 'spiral'
default_script_options = {'power' : 100}
