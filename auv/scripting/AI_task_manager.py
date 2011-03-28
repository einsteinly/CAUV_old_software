import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import AI_messages

task_list = {
            'pipe': ['pipe_detect','pipe_confirm','pipe_follow'],
            }

class taskManager():
    def __init__(self):
        self.node = cauv.node.Node('pytskcon')
        self.ai_node = AI_messages.ai_node(self.node, process_initial='t')
        self.available_tasks
        for task in task_list:
            self.setup_task(task, *task_list[task])
    def setup_task(task_name, task_detector, task_confirmer, task_action):
        #start detectors
        self.ai_node.send('d','start',task_detector)
        self.
    def run(self):
        #check for messages
        #react to messages
        #sleep

if __name__ == '__main__':
    tm = taskManager()
    tm.run()
