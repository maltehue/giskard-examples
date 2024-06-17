import os
import psutil
import subprocess
import rospy
from IPython.display import display, HTML
from sidecar import Sidecar

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
import threading
import random

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
    run_command('roslaunch rvizweb update_config.launch config_file:=$PWD/../launch/rvizweb_config/tracy.json')

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

# Create custom marker for visualization
def create_marker(marker_id,
                  position=Point(0, 0, 0),
                  scale=Point(1, 1, 1),
                  orientation=Quaternion(0, 0, 0, 1),
                  color=ColorRGBA(1, 1, 0, 1), 
                  frame_id="tracy/world"
                 ):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "tracy_playground"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.scale = scale
    marker.action = Marker.ADD
    marker.pose.orientation = orientation
    marker.pose.position = position
    marker.color = color
    marker.lifetime = rospy.Duration()
    return marker

# Publish marker array
def publish_marker_array(marker_array, topic='visualization_marker_array'):
    marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=100)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rate.sleep()
    

# A tracy demo class
class TracyDemo:
    def __init__(self, colors):
        self.topic = 'demo/visualization_marker_array'
        self.colors = colors
        self.color_block_size = 0.3
        self.color_position_r = [
            Point(0.7 + i % 2 * self.color_block_size, -0.3 - int(i / 2) * self.color_block_size, 0.01) for i in range(len(self.colors))
        ]
        self.color_position_l = [
            Point(0.7 + i % 2 * self.color_block_size, 0.3 + int(i / 2) * self.color_block_size, 0.01) for i in range(len(self.colors))
        ]
        
    # 
    def setup_color_blocks(self):
        """Display color blocks on the tracy table"""
        marker_array = MarkerArray()
        marker_array.markers = []
        for i in range(len(self.colors)):
            marker_array.markers.append(create_marker(
                marker_id=i,
                position=self.color_position_r[i],
                scale=Point(self.color_block_size, self.color_block_size, 0.01),
                color=self.colors[i]
            ))
            marker_array.markers.append(create_marker(
                marker_id=i + len(self.colors),
                position=self.color_position_l[i],
                scale=Point(self.color_block_size, self.color_block_size, 0.01),
                color=self.colors[i]
            ))
        # publish ros message in the background
        bg_thread = threading.Thread(target=publish_marker_array, args=(marker_array, self.topic))
        bg_thread.daemon = True  # This makes sure the thread will exit when the main program exits
        bg_thread.start()

    def get_color_pos(self, color, hand='r'):
        """Get the the position of a color block (placeholder of perception)"""
        index = self.colors.index(color)
        if hand == 'r':
            pos = self.color_position_r[index]
        else:
            pos = self.color_position_l[index]
        # Copy the postion, not modify it.
        pos = Point(pos.x, pos.y, pos.z)
        # add some bias to the target position
        pos.x = pos.x + random.uniform(-self.color_block_size, self.color_block_size) * 0.3
        pos.y = pos.y + random.uniform(-self.color_block_size, self.color_block_size) * 0.3
        return pos
