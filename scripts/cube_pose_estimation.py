import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Initialize ROS Node
rospy.init_node('object_detection_node')

# Initialize CvBridge
bridge = CvBridge()

# Camera intrinsic parameters (replace with your camera parameters)
fx = 600.0
fy = 600.0
cx = 320.0
cy = 240.0
box_width = 0.0254
box_height = 0.0254

# Flag to indicate whether CameraInfo has been received
camera_info_received = False
camera_matrix = None
dist_coeffs = None
cube_points_3D = [(0, 0, box_height), (box_width, 0, box_height), (box_width, 0, 0), (box_width, box_width, 0), (0, box_width, 0), (0, box_width, box_height)]
# Callback function to process RGB images
def image_callback(rgb_msg):
    global camera_info_received, camera_matrix, dist_coeffs
    # Physical dimensions of the box (replace with the actual dimensions in meters)

    cube_points_3D = [(0, 0, box_height), (box_width, 0, box_height), (box_width, 0, 0), (box_width, box_width, 0), (0, box_width, 0), (0, box_width, box_height)]
    corners_2d = [(0,0), (0,1), (1,1), (1,0), (0,1), (1,1)]
    # try:
    # Convert RGB message to OpenCV image
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

    # Check if CameraInfo has been received
    if not camera_info_received:
        rospy.logwarn("CameraInfo not received yet. Skipping pose estimation.")
        return

    # Convert RGB to HSV
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    # Define color range for the orange box
    lower_color = np.array([5, 100, 100], dtype=np.uint8)  # Lower bound for orange in HSV
    upper_color = np.array([50, 255, 255], dtype=np.uint8)  # Upper bound for orange in HSV

    # Masking based on color
    mask = cv2.inRange(hsv_image, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Assume the largest contour is the box
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        # Create an empty mask to draw the largest contour
        largest_contour_mask = np.zeros_like(mask)

        # Draw the largest contour on the mask
        cv2.drawContours(largest_contour_mask, [largest_contour], -1, (255), thickness=cv2.FILLED)

        edges = cv2.Canny(largest_contour_mask, 100, 200)
        # cv2.imshow("edges", edges)

        # Approximate the contour and get the corners
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        corners = cv2.approxPolyDP(largest_contour, epsilon, True)

        # Draw the corners on the image
        corner_names = ['(0, 0, 1)', '(1, 0, 1)', '(1, 0, 0)', '(1, 1, 0)', '(0, 1, 0)', '(0, 1, 1)']
        for i, corner in enumerate(corners):
            if(i<=5):
                x, y = corner.ravel()
                x = int(x)
                y = int(y)
                corners_2d[i] = (x, y)
                cv2.circle(rgb_image, (x, y), 4, (0, 0, 255), -1)
                cv2.putText(rgb_image, corner_names[i], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                if(i==5):

                    corners_2d = np.float32(corners_2d)
                    corners_3d = np.float32(cube_points_3D)

                    # Calculate the distances between each pair of adjacent corners
                    distances = [np.linalg.norm(corners_2d[i] - corners_2d[(i+1)%6]) for i in range(6)]

                    # Check if the distances are similar
                    tolerance = 0.03  # Adjust this value as needed
                    mean_distance = np.mean(distances)
                    if all(abs(distance - mean_distance)//mean_distance <= tolerance for distance in distances):
                        # The distances are similar, so proceed with the pose estimation
                        if len(corners_2d) >= 4 and len(corners_3d) >= 4 and len(corners_2d) == len(corners_3d):
                            _, rvec, tvec = cv2.solvePnP(corners_3d, corners_2d, camera_matrix, dist_coeffs)

                           # Convert rotation vector to rotation matrix
                            R, _ = cv2.Rodrigues(rvec)
                            pub_tf(R, tvec)

                        else:
                            rospy.logwarn("len(corners_2d), len(corners_3d): {}, {}".format(len(corners_2d), len(corners_3d)))

                    else:
                        rospy.logwarn("The distances between the corners are not similar, skipping pose estimation...")
                        rospy.loginfo("the mean distance is: {}".format(mean_distance))
                        rospy.loginfo("the distances are: {}".format(distances))

                    x, y, w, h = cv2.boundingRect(largest_contour)

                    # Get the center of the bounding box (this can be used as the pose)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Print or use the pose information as needed
                    # rospy.loginfo("Detected box at (x={}, y={})".format(center_x, center_y))

                    # Draw the bounding box on the RGB image
                    cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Draw the center
                    cv2.circle(rgb_image, (center_x, center_y), 5, (255, 0, 0), -1)

    # Display the result (you can replace this with publishing the result to a ROS topic)
    result_image = np.hstack((rgb_image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow("Result", result_image)
    cv2.waitKey(1)

    # except Exception as e:
    #     # pass
    #     rospy.logerr("Error processing RGB image: {}".format(e))

# Callback function to process CameraInfo
def camera_info_callback(camera_info_msg):
    global camera_info_received, camera_matrix, dist_coeffs
    # Use CameraInfo only if it hasn't been received before
    if not camera_info_received:
        camera_matrix = np.array(camera_info_msg.K).reshape((3, 3))
        dist_coeffs = np.array(camera_info_msg.D)
        rospy.loginfo("CameraInfo received. Ready to estimate pose.")
        camera_info_received = True

def PoseEstimation(rgb_image, corners):
    cube_points_2D = np.float32([corners[0], corners[1], corners[2], corners[3]])

    # Solve the PnP problem
    _, rotation_vector, translation_vector = cv2.solvePnP(cube_points_3D, cube_points_2D, camera_matrix, None)

    # Draw the axis at the center of the cube
    axis = np.float32([[box_width, 0, 0], [0, box_width, 0], [0, 0, -box_width]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, None)
    corner = tuple(corners[0].ravel())
    img = cv2.line(rgb_image, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)


def pub_tf(R, tvec):
    R_4x4 = np.eye(4)
    R_4x4[:3, :3] = R
    # Create a TransformStamped message
    transform = TransformStamped()

    # Fill in the header
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "camera_color_optical_frame"  # The name of the parent frame
    transform.child_frame_id = "box"  # The name of the child frame

    # Fill in the translation
    transform.transform.translation.x = float(tvec[0])
    transform.transform.translation.y = float(tvec[1])
    transform.transform.translation.z = float(tvec[2])

    # Convert the rotation matrix to a quaternion
    quaternion = tf.transformations.quaternion_from_matrix(R_4x4)
    # rospy.loginfo("Publishing transform: {}".format(tvec))

    # Fill in the rotation
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    rospy.loginfo("box pose: {}".format(transform))
    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(transform)

def draw_axis(rotation_vector, translation_vector, rgb_image, corner):
    # Define the axis (a 3D array corresponding to 3 lines in the x, y, and z directions)
    axis = np.float32([[box_width/2, 0, 0], [0, box_width/2, 0], [0, 0, -box_width/2]]).reshape(-1, 3)
    corner_x = int(corner[0])
    corner_y = int(corner[1])
    # Project the 3D points to the image plane
    imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
    # rospy.loginfo("imgpts: %s", corner)
    # Draw the lines (from the center of the cube to the projected points)
    img = cv2.line(rgb_image, (corner_x, corner_y), tuple(imgpts[2].ravel().astype(int)), (255,0,0), 5)
    img = cv2.line(img, (corner_x, corner_y), tuple(imgpts[1].ravel().astype(int)), (0,255,0), 5)
    img = cv2.line(img, (corner_x, corner_y), tuple(imgpts[0].ravel().astype(int)), (0,0,255), 5)

# Subscribe to RGB and CameraInfo topics
rgb_topic = '/camera/color/image_raw'
camera_info_topic = '/camera/color/camera_info'
rospy.Subscriber(rgb_topic, Image, callback=image_callback, queue_size=1)
rospy.Subscriber(camera_info_topic, CameraInfo, callback=camera_info_callback, queue_size=1)

# Spin ROS
rospy.spin()
