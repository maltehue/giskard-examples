from py_trees.behaviours import failure

from giskardpy_ros.python_interface.python_interface import GiskardWrapper
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_matrix
from giskardpy.model.world_config import WorldWithOmniDriveRobot
from giskardpy.casadi_wrapper import TransMatrix
from pyswip import Prolog, registerForeign
import rospy
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def setup_demo():
    rospy.init_node('test')
    giskard = GiskardWrapper()
    prolog = Prolog()

    marker_pub = rospy.Publisher("/trajectory_markers", MarkerArray, queue_size=10)

    def joint_trajectory_to_marker_array(msg: JointTrajectory, frame_id: str = "map") -> MarkerArray:
        marker_array = MarkerArray()

        # Trajectory line marker
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # Line thickness
        line_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
        line_marker.pose.orientation.w = 1.0  # Identity quaternion (no rotation)

        # Sphere markers for each trajectory point
        point_markers = []

        for idx, point in enumerate(msg.points):
            if len(point.positions) < 3:
                rospy.logwarn("Trajectory points must have at least 3 values (x, y, z). Skipping point.")
                continue
            x, y, z = point.positions[:3]  # Assume first 3 positions are x, y, z
            # Add to the line strip
            line_marker.points.append(Point(x, y, z))
        # Add all markers to the MarkerArray
        marker_array.markers.append(line_marker)

        return marker_array

    # setup kitchen environment
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1
    r = giskard.world.add_urdf(name='dlr_kitchen', urdf=rospy.get_param('kitchen_description'), pose=pose)

    # setup initial pose of the robot
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position = Point(1, 2, 0)
    pose.pose.orientation = Quaternion(0, 0, 0.7071068, 0.7071068)
    giskard.motion_goals.add_cartesian_pose(pose, 'base_link', 'map')
    giskard.add_default_end_motion_conditions()
    giskard.motion_goals.allow_all_collisions()
    r = giskard.execute()

    # create local giskard world model
    urdf = open('pr2_local.urdf', 'r').read()
    config = WorldWithOmniDriveRobot(urdf=urdf)
    with config.world.modify_world():
        config.setup()
    config.world.register_controlled_joints(config.world.movable_joint_names)

    t = TransMatrix()
    with config.world.modify_world():
        config.world.add_urdf(open('dlr_kitchen.urdf', 'r').read(), parent_link_name='map', pose=t)

    # foreign functions
    def execute():
        giskard.motion_goals.allow_all_collisions()
        giskard.execute()

    def add_init_pose():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(1, 2, 0)
        pose.pose.orientation = Quaternion(0, 0, 0.7071068, 0.7071068)
        giskard.motion_goals.add_cartesian_pose(pose, 'base_link', 'map')
        giskard.add_default_end_motion_conditions()

    def traj_open_container(motion, joint, goalState, handle):
        if str(motion) != 'envJointGoal':
            return False
        if isinstance(joint, bytes):
            joint = joint.decode('utf-8')
        if isinstance(handle, bytes):
            handle = joint.decode('utf-8')
        # marker_pub.publish(MarkerArray())

        giskard.motion_goals.add_joint_position_return_traj(goal_state={str(joint): float(goalState)}, root_link='map',
                                                            traj_frame=str(handle))
        mon2 = giskard.monitors.add_joint_position(goal_state={str(joint): float(goalState)})

        loc = giskard.monitors.add_local_minimum_reached()
        giskard.monitors.add_end_motion(mon2)
        giskard.monitors.add_cancel_motion(loc, Exception('local min'))
        res = giskard.projection()

        marker_pub.publish(joint_trajectory_to_marker_array(res.trajectory))

        if not res.error.type == "Exception":
            # feedback.unify('success')
            return True
        else:
            # feedback.unify(str('failure'))
            return False

    def add_open_container(motion, joint, goalState, handle, gripper):
        if str(motion) != 'envJointGoal':
            return False
        if isinstance(joint, bytes):
            joint = joint.decode('utf-8')
        pose = PoseStamped()
        pose.header.frame_id = str(handle)
        pose.pose.position = Point(0, 0, 0)
        pose.pose.orientation.w = 1

        # mon1 = giskard.monitors.add_cartesian_pose(root_link='base_link', tip_link='r_gripper_tool_frame', goal_pose=pose, position_threshold=0.03)
        giskard.motion_goals.add_cartesian_pose(pose, str(gripper), 'map')
        giskard.add_default_end_motion_conditions()
        giskard.motion_goals.allow_all_collisions()
        giskard.execute()

        giskard.motion_goals.add_open_container(tip_link=str(gripper), environment_link=str(handle),
                                                goal_joint_state=float(goalState))
        mon2 = giskard.monitors.add_joint_position(goal_state={str(joint): float(goalState)})

        giskard.monitors.add_end_motion(mon2)
        loc = giskard.monitors.add_local_minimum_reached()
        giskard.monitors.add_cancel_motion(loc, Exception('local min'))

    def giskard_project_eval():
        giskard.motion_goals.allow_all_collisions()
        res = giskard.projection()

        if not res.error.type == "Exception":
            # feedback.unify('success')
            return True
        else:
            # feedback.unify(str('failure'))
            return False

    registerForeign(add_init_pose, arity=0)
    registerForeign(add_open_container, arity=5)
    registerForeign(execute, arity=0)
    registerForeign(giskard_project_eval, arity=0)
    registerForeign(traj_open_container, arity=4)

    def hasArticulation(joint, link):
        tip = config.world.search_for_link_name(str(link))
        result = config.world.get_movable_parent_joint(tip).short_name
        if result:
            joint.unify(str(result))
            return True
        return False

    def partOf(link1, link2):
        l1 = config.world.search_for_link_name(str(link1))
        l2 = config.world.search_for_link_name(str(link2))
        results = config.world.get_links_in_branch_of_link(l1)
        return l2 in results

    registerForeign(hasArticulation, arity=2)
    registerForeign(partOf, arity=2)

    prolog.consult("kb.pl")

    return prolog