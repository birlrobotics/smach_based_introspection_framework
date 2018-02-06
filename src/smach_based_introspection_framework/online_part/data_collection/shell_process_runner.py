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
        self.process = subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(1)
        if self.process.poll() is not None:
            raise Exception("Process died immediately. returncode: %s. stdout: %s. stderr: %s"%(
                self.process.returncode,
                self.process.stdout.read(),
                self.process.stderr.read(),
            ))
        self.started = True


    def stop(self):
        if not self.started:
            return
        ppid = self.process.pid 
        parent = psutil.Process(ppid)
        for child in parent.children(recursive=True):
            child.send_signal(signal.SIGINT)
            child.wait()
        parent.send_signal(signal.SIGINT)
        parent.wait()
        self.started = False 
