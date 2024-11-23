import subprocess
import os
import signal

class RosWebsocketManager:
    def __init__(self):
        self.process = None

    def start_roslaunch(self):
        if self.process is None or self.process.poll() is not None:
            # roslaunch_cmd = ["roslaunch", "rosbridge_server", "rosbridge_websocket.launch"]

            self.process = subprocess.Popen(
                'roslaunch rosbridge_server rosbridge_websocket.launch',
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            return f"Roslaunch started with PID: {self.process.pid}"
        else:
            return "Roslaunch is already running."

    def stop_roslaunch(self):

        subprocess.Popen(
                'rosnode kill rosbridge_websocket',
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                # preexec_fn=os.setsid,
                text=True)

        subprocess.Popen(
                'rosnode kill rosapi',
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                # preexec_fn=os.setsid,
                text=True)    

        if self.process is not None and self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait()
            self.process = None
            return "Roslaunch stopped successfully."
        else:
            return "Roslaunch is not running."

