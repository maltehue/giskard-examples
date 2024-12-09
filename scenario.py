from giskardpy.python_interface.python_interface import GiskardWrapper
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_matrix
from copy import deepcopy


def run_demo():
    rospy.init_node('scenario')
    giskard = GiskardWrapper()

    giskard.clear_motion_goals_and_monitors()
    giskard.world.clear()

    better_pose_tracy = {
        'left_shoulder_pan_joint': 2.539670467376709,
        'left_shoulder_lift_joint': -1.46823854119096,
        'left_elbow_joint': 2.1197431723224085,
        'left_wrist_1_joint': -1.4825000625899811,
        'left_wrist_2_joint': 5.467689037322998,
        'left_wrist_3_joint': -0.9808381239520472,
        'right_shoulder_pan_joint': 3.7588136196136475,
        'right_shoulder_lift_joint': -1.7489210567870082,
        'right_elbow_joint': -2.054229259490967,
        'right_wrist_1_joint': -1.6140786610045375,
        'right_wrist_2_joint': 0.7295855283737183,
        'right_wrist_3_joint': 3.944669485092163,
        'left_finger_joint': 0
    }
    giskard.motion_goals.add_joint_position(better_pose_tracy)
    giskard.add_default_end_motion_conditions()
    giskard.motion_goals.allow_all_collisions()
    giskard.execute()

    sbs_pose = PoseStamped()
    sbs_pose.header.frame_id = 'map'
    sbs_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[1, 0, 0, 0],
                                                                    [0, 0, -1, 0],
                                                                    [0, 1, 0, 0],
                                                                    [0, 0, 0, 1]]))
    sbs_pose.pose.position = Point(1.9, 0, 0.91)
    giskard.world.add_mesh(name='sbs', mesh='data/Falcon6erSBS.STL', pose=sbs_pose,
                           scale=(0.001, 0.001, 0.001))

    cold_pose = PoseStamped()
    cold_pose.header.frame_id = 'map'
    cold_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[1, 0, 0, 0],
                                                                     [0, 0, -1, 0],
                                                                     [0, 1, 0, 0],
                                                                     [0, 0, 0, 1]]))
    cold_pose.pose.position = Point(1.9, 0.5, 0.91)
    giskard.world.add_mesh(name='coldplate', mesh='data/coldplate.stl',
                           pose=cold_pose,
                           scale=(0.001, 0.001, 0.001))

    giskard.motion_goals.add_joint_position({'robot_arm_gripper_joint': 0.066})
    # ------------- fixed arm manipulation while kevin sets up -----------
    cyl_pose = PoseStamped()
    cyl_pose.header.frame_id = 'l_gripper_tool_frame'
    cyl_pose.pose.orientation.w = 1
    cyl_pose.pose.position.z = 0.025
    giskard.world.add_cylinder('glass1', 0.05, 0.01, cyl_pose, 'l_gripper_tool_frame')

    insert_pose = PoseStamped()
    insert_pose.header.frame_id = 'sbs'
    insert_pose.pose.position.x = 0.02
    insert_pose.pose.position.y = 0.2
    insert_pose.pose.position.z = 0.025
    insert_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[1, 0, 0, 0],
                                                                       [0, 0, -1, 0],
                                                                       [0, 1, 0, 0],
                                                                       [0, 0, 0, 1]]))
    joint_mon = giskard.monitors.add_joint_position({'left_finger_joint': 0.6})
    giskard.motion_goals.add_joint_position({'left_finger_joint': 0.6}, end_condition=joint_mon, name='closeArm')
    insert_mon1 = giskard.monitors.add_cartesian_pose(root_link='map', tip_link='l_gripper_tool_frame',
                                                      goal_pose=insert_pose, name='insert1', start_condition=joint_mon)
    giskard.motion_goals.add_cartesian_pose(insert_pose, 'l_gripper_tool_frame', 'map', name='insertGoal1',
                                            end_condition=insert_mon1, start_condition=joint_mon)
    insert_pose2 = deepcopy(insert_pose)
    insert_pose2.pose.position.y = 0.05
    giskard.motion_goals.add_cartesian_pose(insert_pose2, 'l_gripper_tool_frame', 'map', name='insertGoal2',
                                            start_condition=insert_mon1)
    insert_mon2 = giskard.monitors.add_cartesian_pose(root_link='map', tip_link='l_gripper_tool_frame',
                                                      goal_pose=insert_pose2, name='insert2',
                                                      start_condition=insert_mon1)
    # --------------------------------------------------------------------
    base_goal = PoseStamped()
    base_goal.header.frame_id = 'map'
    base_goal.pose.position = Point(1, 0, 0)
    base_goal.pose.orientation.w = 1
    giskard.motion_goals.add_diff_drive_base(goal_pose=base_goal, tip_link='robot_base_footprint', root_link='map')

    hotel_pose = PoseStamped()
    hotel_pose.header.frame_id = 'robot_top_3d_laser_link'
    hotel_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 1, 0, 0],
                                                                      [0, 0, -1, 0],
                                                                      [-1, 0, 0, 0],
                                                                      [0, 0, 0, 1]]))
    hotel_pose.pose.position.z = 0.4
    giskard.motion_goals.add_cartesian_pose(hotel_pose, 'robot_arm_tool_link', 'robot_arm_base_link')
    kevin_mon = giskard.monitors.add_cartesian_pose(root_link='map', tip_link='robot_base_footprint',
                                                    goal_pose=base_goal, name='kevin_init')

    giskard.motion_goals.allow_all_collisions()
    # giskard.add_default_end_motion_conditions()
    giskard.monitors.add_end_motion(f'{insert_mon2} and {kevin_mon}')
    giskard.execute()
    # ---------- MOVE BACK ARM ----------------------
    giskard.world.update_parent_link_of_group(name='glass1', parent_link='sbs')
    insert_pose.pose.position.x = 0.4
    insert_pose.pose.position.z = 0.4
    insert_pose.pose.position.y = 0.3
    joint_mon = giskard.monitors.add_joint_position({'left_finger_joint': 0.1})
    giskard.motion_goals.add_joint_position({'left_finger_joint': 0.1}, end_condition=joint_mon, name='closeArm')
    giskard.motion_goals.add_cartesian_pose(insert_pose, 'l_gripper_tool_frame', 'map', name='retreat',
                                            start_condition=joint_mon)
    giskard.add_default_end_motion_conditions()
    giskard.motion_goals.allow_all_collisions()
    giskard.execute()

    # --------- KEVIN MANIPULATION starts after this -------------

    goal_pose = sbs_pose
    goal_pose.pose.orientation.w = 1
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.z = 0
    goal_pose.pose.position.y -= 0.043
    goal_pose.pose.position.z += 0.2
    giskard.motion_goals.add_cartesian_pose(goal_pose, 'robot_arm_tool_link', 'map', reference_angular_velocity=1)
    giskard.motion_goals.allow_all_collisions()
    # giskard.motion_goals.avoid_all_collisions()
    # giskard.motion_goals.allow_self_collision()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    goal_pose.pose.position.z -= 0.19
    giskard.motion_goals.add_cartesian_pose(goal_pose, 'robot_arm_tool_link', 'map')
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    giskard.motion_goals.add_joint_position({'robot_arm_gripper_joint': 0.049})  # close
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    giskard.world.update_parent_link_of_group(name='sbs', parent_link='robot_arm_tool_link')

    goal_pose.pose.position.z += 0.1
    giskard.motion_goals.add_cartesian_pose(goal_pose, 'robot_arm_tool_link', 'map')
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    hotel_pose = PoseStamped()
    hotel_pose.header.frame_id = 'robot_top_3d_laser_link'
    hotel_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 1, 0, 0],
                                                                      [0, 0, -1, 0],
                                                                      [-1, 0, 0, 0],
                                                                      [0, 0, 0, 1]]))
    hotel_pose.pose.position.z = 0.4
    giskard.motion_goals.add_cartesian_pose(hotel_pose, 'robot_arm_tool_link', 'robot_arm_base_link')
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    base_goal = PoseStamped()
    base_goal.header.frame_id = 'map'
    base_goal.pose.position = Point(0.8, 0.4, 0)
    base_goal.pose.orientation.w = 1
    giskard.motion_goals.add_diff_drive_base(goal_pose=base_goal, tip_link='robot_base_footprint', root_link='map')
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    goal_pose2 = cold_pose
    goal_pose2.pose.orientation.w = 1
    goal_pose2.pose.orientation.x = 0
    goal_pose2.pose.orientation.y = 0
    goal_pose2.pose.orientation.z = 0
    goal_pose2.pose.position.x -= 0.05
    goal_pose2.pose.position.z += 0.2
    giskard.motion_goals.add_cartesian_pose(goal_pose2, 'robot_arm_tool_link', 'map', reference_angular_velocity=1)
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    goal_pose2.pose.position.x -= 0.01
    goal_pose2.pose.position.z -= 0.12
    giskard.motion_goals.add_cartesian_pose(goal_pose2, 'robot_arm_tool_link', 'map', reference_angular_velocity=1)
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()

    giskard.world.update_parent_link_of_group(name='sbs', parent_link='map')
    giskard.motion_goals.add_joint_position({'robot_arm_gripper_joint': 0.066})
    goal_pose2.pose.position.z += 0.2
    giskard.motion_goals.add_cartesian_pose(goal_pose2, 'robot_arm_tool_link', 'map')
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    giskard.execute()
