import subprocess
import os
import signal
import psutil
import time

class ShellProcessRunner(object):
    def __init__(self):
        self.cmd = None
        self.started = False

    def start(self):
        if self.started:
            return

        if self.cmd is None:
            raise Exception("Process cmd is None.")
        self.process = subprocess.Popen(self.cmd, shell=True)
        if self.process.poll() is not None:
            raise Exception("Process died immediately. returncode: %s."%(
                self.process.returncode,
            ))
        self.started = True


    def stop(self):
        if not self.started:
            return
        ppid = self.process.pid 
        parent = psutil.Process(ppid)
        for child in parent.children(recursive=True):
            try:
                child.send_signal(signal.SIGINT)
                child.wait()
            except psutil.NoSuchProcess:
                pass
        parent.send_signal(signal.SIGINT)
        parent.wait()
        self.started = False 

    def check_if_successfully_start(self):
        # TODO: implement custom ways to check successful start
        pass
