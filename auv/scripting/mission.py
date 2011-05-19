from AI_classes import detectorCondition, aiTask

task_list = [
            aiTask('test', 1, conditions = [detectorCondition('test_detector', 'test'),]),
            #aiTask('pipe', 1, conditions = [detectorCondition('pipe_detector', 'pipe'),]),
            ]
default_script = 'spiral'
