from AI_classes import detectorCondition, aiTask

task_list = [
            #aiTask('test', 1, conditions = [detectorCondition('test_detector', 'test'),]),
            #aiTask('pipe', 1, conditions = [detectorCondition('pipe_detector', 'pipe'),]),
            aiTask('circle_buoy', 1, conditions = [detectorCondition('buoy_detector', 'buoy_detector'),]),
            aiTask('avoid_collision', 1, conditions = [detectorCondition('collision_detector', 'collision_detector'),]),
            ]
default_script = 'test'
