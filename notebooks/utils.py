import os
import psutil
import subprocess
import rospy
from IPython.display import display, HTML
from sidecar import Sidecar

# Directory of the ROS launch files
LAUNCH_FILE_DIR = os.path.abspath(os.path.join(os.getcwd(), "../launch"))

# To display visualization tools on the left sidecar
SIDECAR = {
    'rvizweb': None,
    'desktop': None
}

# To manage the roslaunch process in the background
LAUNCH_PROCESS = None

# Check if it is running on binderhub
try:
    JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
except KeyError:
    JUPYTERHUB_USER = None

# Display iframe with a resizable widget
def resizable_iframe(url):
    return HTML(f"""
        <div class="iframe-widget" style="width: calc(100% + 10px);">
            <iframe src="{url}" width="100%", height="100%">
        </div>
    """)

# Open rvizweb on a sidecar
def open_rvizweb():
    # Close the sidecar if it is already open
    if SIDECAR['rvizweb'] is not None:
        SIDECAR['rvizweb'].close()
    try:
        SIDECAR['rvizweb'] = Sidecar(title='RVIZWEB', anchor='right')
        with SIDECAR['rvizweb']:
            display(resizable_iframe(rospy.get_param('/rvizweb/jupyter_proxy_url')))
    except Exception as e:
        print('Can not fetch rvizweb url.')

# Open Desktop
def open_desktop():
    if SIDECAR['desktop'] is not None:
        SIDECAR['desktop'].close()
    SIDECAR['desktop'] = Sidecar(title='desktop', anchor='right')
    url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
    with SIDECAR['desktop']:
        display(resizable_iframe(f"{url_prefix}/desktop"))

# Execute the roslaunch command in the background
def run_command(cmd, background=True):
    global LAUNCH_PROCESS
    if LAUNCH_PROCESS is not None:
        LAUNCH_PROCESS.terminate()
        LAUNCH_PROCESS.wait()
    command = ['/bin/bash', '-c', cmd]
    LAUNCH_PROCESS = psutil.Popen(command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
