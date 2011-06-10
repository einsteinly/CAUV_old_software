from AI_classes import detectorCondition, aiTask

#a dict of tasks that can be added or removed as neccessary
task_list = {
            'test': aiTask('test', 1, detectors_enabled=False, conditions = [detectorCondition('test_detector', 'test'),]),
            'pipe': aiTask('pipe', 1, conditions = [detectorCondition('pipe_detector', 'pipe'),]),
            'circle_buoy': aiTask('circle_buoy', 1, conditions = [detectorCondition('buoy_detector', 'buoy_detector'),]),
            }

#a list of tasks to look out for at the beggining
initial_tasks = [
                'test',
                ]
#script that runs otherwise
default_script = 'spiral'
