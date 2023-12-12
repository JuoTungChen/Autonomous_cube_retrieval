from Lib.ur5.FunctionLibrary import FunctionLib
import rospy
import time, actionlib,sys,tf, tf2_ros, copy
from tf import TransformListener
import numpy as np
import geometry_msgs.msg, control_msgs

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

def get_box_location():
    start_time = time.time()
    timeout = 15
    while time.time() - start_time <= timeout and not rospy.is_shutdown(): 
        try:
            lib.tf_listener.waitForTransform('/world', '/box', rospy.Time(0), rospy.Duration(3.0))
            trans, rot = lib.tf_listener.lookupTransform('/world', '/box', rospy.Time(0))
            eul = lib.quaternion_to_euler(rot[0], rot[1], rot[2], rot[3])

            if trans[2] < -0.11:
                trans[2]= -0.11


            if eul[0] > 0:
                eul[0] -= 180
            else:
                eul[0] += 180

            z_offset = -0.00
            r = lib.euler_to_quaternion(eul[0], eul[1], eul[2])
            return [trans[0], trans[1], trans[2], r[0], r[1], r[2], r[3]]
        except (tf2_ros.TransformException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("box not found. Waiting for box to appear")
            rospy.sleep(1.0)
    rospy.logwarn("Timeout occurred while waiting for box to appear")
    exit()


def add_cube_to_workspace(name,x,y,z,sx,sy,sz,ox,oy,oz,ow):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.orientation.w = ow
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    lib.scene.add_box(name, pose, size=(sx,sy,sz))

box_dim = 0.0254
box_location = get_box_location()
ee_pose = lib.get_current_end_effector_pose()

lib.go(box_location[0] + box_dim, box_location[1] - box_dim, box_location[2] + 0.1, ee_pose[3], ee_pose[4], ee_pose[5], orientation_constraint=False, position_constraint=False, joint_constraint=False, velocity=1.0, acceleration=0.8)
rospy.sleep(1)

lib.go(box_location[0] + box_dim, box_location[1] - box_dim, box_location[2] + 0.1 - box_dim, ee_pose[3], ee_pose[4], ee_pose[5], orientation_constraint=False, position_constraint=False, joint_constraint=False, velocity=1.0, acceleration=0.8)
rospy.sleep(1)

lib.close_gripper("cube")

# move to a random drop location
lib.go(box_location[0], box_location[1] + box_dim, box_location[2] + 0.2, ee_pose[3], ee_pose[4], ee_pose[5], orientation_constraint=False, position_constraint=False, joint_constraint=False, velocity=1.0, acceleration=0.8)

lib.open_gripper()

# move to original position
lib.go(ee_pose[0], ee_pose[1], ee_pose[2], ee_pose[3], ee_pose[4], ee_pose[5])


