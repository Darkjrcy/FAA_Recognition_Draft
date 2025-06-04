import subprocess
import time
import os, signal
p1 = subprocess.Popen('ros2 launch gnss_multipath_plugin purdue.launch.py', shell=True)
p2 = subprocess.Popen('python generate_heatmap.py', shell=True)
time.sleep(10.0)
os.killpg(os.getpgid(p1.pid), signal.SIGTERM)
os.killpg(os.getpgid(p2.pid), signal.SIGTERM)
