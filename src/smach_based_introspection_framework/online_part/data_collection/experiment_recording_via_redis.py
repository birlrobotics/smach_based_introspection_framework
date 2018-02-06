from shell_process_runner import ShellProcessRunner
import os 

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
class RedisRecord(ShellProcessRunner):
    def __init__(self):
        super(RedisRecord, self).__init__()
        script_path = os.path.join(
            dir_of_this_script,
            'topic_multimodal_online_recorder.py',
        )
        self.cmd = "python %s"%os.path.realpath(script_path)
