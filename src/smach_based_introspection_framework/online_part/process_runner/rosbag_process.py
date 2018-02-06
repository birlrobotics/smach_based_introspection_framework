import os
from shell_process_runner import ShellProcessRunner

class RosbagRecord(ShellProcessRunner):
    def __init__(self, bag_path, list_of_topics):
        super(RosbagRecord, self).__init__()
        self.cmd = "rosbag record -O %s %s"%(os.path.realpath(bag_path), ' '.join(list_of_topics))
