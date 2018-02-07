from shell_process_runner import ShellProcessRunner
import os 

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
class AnomalyClassificationProc(ShellProcessRunner):
    def __init__(self):
        super(AnomalyClassificationProc, self).__init__()
        script_path = os.path.join(
            dir_of_this_script,
            'scripts',
            'redis_based_anomaly_classification.py',
        )
        self.cmd = "python %s"%os.path.realpath(script_path)
