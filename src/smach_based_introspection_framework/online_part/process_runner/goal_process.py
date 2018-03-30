from shell_process_runner import ShellProcessRunner
import os 

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
class GoalProc(ShellProcessRunner):
    def __init__(self):
        super(GoalProc, self).__init__()
        script_path = os.path.join(
            dir_of_this_script,
            'scripts',
            'goal_topic_and_service.py',
        )
        self.cmd = "python %s"%os.path.realpath(script_path)
