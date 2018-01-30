import subprocess
import os
import signal
import psutil

class RosbagRecord(object):
    def __init__(self, bag_path, list_of_topics):
        self.cmd = "rosbag record -O %s %s"%(os.path.realpath(bag_path), ' '.join(list_of_topics))
        self.started = False

    def start(self):
        if self.started:
            return
        self.process = subprocess.Popen(self.cmd, shell=True)
        self.started = True

    def stop(self):
        if not self.started:
            return
        ppid = self.process.pid 
        parent = psutil.Process(ppid)
        for child in parent.children(recursive=True):
            child.send_signal(signal.SIGINT)
        parent.send_signal(signal.SIGINT)
        self.started = False 
