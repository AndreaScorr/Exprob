"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import OrderedDict

from ros2_aruco import transformations

# ROS Messages
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_msgs.msg import Odometry
#from tf_transformations import euler_from_quaternion

orientation_q = None
position_to_increase = -3.14
msg_position = Float64MultiArray()
msg_position.data = [position_to_increase]

class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        self.declare_parameter("marker_size", .0610)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.pose_orientation = 1
        self.box_marker_dict= {}
        # Make sure we have a valid dictionary id
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"bad aruco_dictionary_id: {dictionary_id_name}")
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(f"valid options: {options}")

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.position = self.create_subscription(Odometry,'odom',self.orientation_and_position_callback, 10)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
        self.image_pub = self.create_publisher(CompressedImage, "/output/image_raw/compressed", 1)
        self.pose_camera_position_pub = self.create_publisher(Float64MultiArray, '/camera_joint_z_axis_controller/commands', 10)
        
        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_parameters.minMarkerPerimeterRate = 0.01  # Adjust this value based on marker size in the image
        self.aruco_parameters.adaptiveThreshConstant = 7  # Adjust adaptive threshold
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same
        self.destroy_subscription(self.info_sub)

    def orientation_and_position_callback(self,msg):
    	 global orientation_q 
    	 orientation_q = msg.pose.pose.orientation
    def image_callback(self, img_msg):
        global position_to_increase
        global msg_position
        
        if(position_to_increase <=3.14):
            position_to_increase += 0.01
            msg_position.data = [position_to_increase]
            self.pose_camera_position_pub.publish(msg_position)
        
        
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
	
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        height_img, width_img = cv_image.shape[:2] #calculate the image dimension
        center_x, center_y = width_img / 2, height_img / 2 #find the center of the image
        
        tolerance= 1
        tol_x = width_img * tolerance / 2
        tol_y = height_img * tolerance / 2
        markers = ArucoMarkers()
        pose_array = PoseArray()
        
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
        
        corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)     
        		
        
        if marker_ids is not None:
           
            	 	
            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
                '''
            for i, marker_id in enumerate(marker_ids):
                if (str(marker_id) not in self.box_marker_dict) and (len(marker_id)==1):
                    self.box_marker_dict.update({str(marker_id): self.pose_orientation})
                    
                
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
	     '''
	     
	     
            # Annotate all markers on the image
            for marker_corners, marker_id in zip(corners, marker_ids):
                
                corners = marker_corners.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
			
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.putText(cv_image, str(marker_id), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.circle(cv_image, (cX, cY), 10, (0, 0, 255), 2)
                             
                
                if( center_x - tol_x <=cX <= center_x + tol_x) and (center_y -tol_y <= cY <= center_y + tol_y): #check if the marker is in the middle of the view
                	#print('more or less in the middle')
                	if (str(marker_id) not in self.box_marker_dict) and(marker_id>0 and marker_id<40):
                         self.box_marker_dict.update({str(marker_id): position_to_increase })
                             #self.box_marker_dict.update({str(marker_id): 1}) #FOR DEBUGGING
                             #print(self.box_marker_dict.keys())
                         self.box_marker_dict = OrderedDict(sorted(self.box_marker_dict.items()))
                         print(self.box_marker_dict.keys())
                             # Display and publish annotated image
                             
                             #create compressed image
                         msg = CompressedImage()
                         msg.header.stamp = self.get_clock().now().to_msg()
                         msg.format = "jpeg"
                         msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
                         self.image_pub.publish(msg) #pubblish the image
                cv2.imshow("Image", cv_image)
                cv2.waitKey(10)
                             
        else:
            corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)            

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

