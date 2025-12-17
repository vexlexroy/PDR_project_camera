import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np


class GtTester(Node):
    def __init__(self):
        super().__init__('gt_video_test')
        self.declare_parameter('distortion_matrix', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.cam_distortion_in = self.get_parameter('distortion_matrix').value
        self.declare_parameter('camera_matrix', [0.0]*4)
        self.cam_matrix_in = self.get_parameter('camera_matrix').value
        self.declare_parameter('marker_size_mm', 10.0)
        self.marker_size_mm = self.get_parameter('marker_size_mm').value
        self.declare_parameter('testing', True)
        self.marker_size_mm = self.get_parameter('testing').value

        self.camera_matrix=np.array([
                          [self.cam_matrix_in[0], 0.0, self.cam_matrix_in[2]],# fx, 0, cx
                          [0.0, self.cam_matrix_in[1], self.cam_matrix_in[3]],# 0, fy, cy
                          [0.0, 0.0, 1.0],
                          ], dtype=np.float32)
        self.distortion_matrix=np.array(self.cam_distortion_in, dtype=np.float32)

        self.last_cam_pose:PoseStamped=None
        self.last_world_cam_T=None
        self.last_frames:list[Image]=[]

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector=cv2.aruco.ArucoDetector(dictionary=self.aruco_dict,detectorParams=self.parameters)
        self.bridge = CvBridge()

        self.marker_image=None

        self.image_sub = self.create_subscription(
            Image,
            '/capture',
            self.proces_images,
            10
        )

        self.camera_pose = self.create_publisher(PoseStamped, '/camera_pose', 10)
        self.marker_image_pub = self.create_publisher(Image, '/marker_image', 10)

        self.path_pub = self.create_publisher(Path, '/camera_path', 10)
        self.camera_path = Path()
        self.camera_path.header.frame_id = 'world'

    def proces_images(self,image_msg:Image): 
        '''gets new image and calculates new camera position
        \ntakes image_msg
        \nno return
        \npublishes new position of camera based on last 2 frames
        '''
        frame= self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if(self.last_frames):
            frame_old= self.bridge.imgmsg_to_cv2(self.last_frames[-1], desired_encoding='bgr8')
            gray_old = cv2.cvtColor(frame_old, cv2.COLOR_BGR2GRAY)
            corners2, ids2, rejected2 = self.detector.detectMarkers(gray_old)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        if(self.last_cam_pose==None and ids is not None and len(ids) > 0):
            self.last_cam_pose=PoseStamped()
            self.last_cam_pose.pose.position.x=0
            self.last_cam_pose.pose.position.y=0
            self.last_cam_pose.pose.position.z=0
            self.last_cam_pose.pose.orientation.x=0
            self.last_cam_pose.pose.orientation.y=0
            self.last_cam_pose.pose.orientation.z=0
            self.last_cam_pose.header.stamp = self.get_clock().now().to_msg()
            self.last_cam_pose.header.frame_id = 'origin'

            self.last_world_cam_T=np.eye(4)
            self.last_frames.append(image_msg)
            cv2.aruco.drawDetectedMarkers(gray,corners,ids)
            self.marker_image = self.bridge.cv2_to_imgmsg(gray, 'mono8')
            self.marker_image_pub.publish(self.marker_image)
            return
        else:
            if(ids is not None and len(ids) > 0):
                matches=[]
                for i,id in enumerate(ids):
                    for j,id2 in enumerate(ids2):
                        if id==id2:
                            matches.append(tuple((i,j)))
                
                if(len(matches)<=0):
                    return

                rvecs, tvecs, _ = self.estimatePoseSingleMarker(
                    corners, self.marker_size_mm/1000, self.camera_matrix, self.distortion_matrix
                )
                rvecs2, tvecs2, _ = self.estimatePoseSingleMarker(
                    corners2, self.marker_size_mm/1000, self.camera_matrix, self.distortion_matrix
                )
                first_match=matches[0]
                T_C1_M = self.make_transform(rvecs2[first_match[0]], tvecs2[first_match[0]])  # marker->camera1
                T_C2_M = self.make_transform(rvecs[first_match[1]], tvecs[first_match[1]])  # marker->camera2
                # Relative transform camera frame1 -> camera frame2
                T_C1_C2 = T_C1_M @ self.invert_transform(T_C2_M) #C2 is next frame and C1 is last frame
                self.last_frames.append(image_msg)
                T_W_C1 = self.last_world_cam_T
                T_W_C2 = T_W_C1 @ T_C1_C2
                self.last_world_cam_T = T_W_C2
                pose = self.matrix_to_pose(T_W_C2)
                self.last_cam_pose=pose
                self.camera_pose.publish(pose)
                
                pose_for_path = PoseStamped()
                pose_for_path.header = pose.header
                pose_for_path.pose = pose.pose
                self.camera_path.header.stamp = self.get_clock().now().to_msg()
                self.camera_path.poses.append(pose_for_path)
                self.path_pub.publish(self.camera_path)
            # get last frame with detected markers,
            # and get average diference of orientation and distance change from all markers 
            # and generate new pose of camera from data (median of data would be nice) 
            cv2.aruco.drawDetectedMarkers(gray,corners,ids)
            self.marker_image = self.bridge.cv2_to_imgmsg(gray, 'mono8')
            self.marker_image_pub.publish(self.marker_image)
            pass


    def destroy_node(self):
        super().destroy_node()



    def make_transform(self,rvec, tvec):
        R, _ = cv2.Rodrigues(rvec.reshape(3,))
        T = np.eye(4, dtype=np.float64)
        T[:3,:3] = R
        T[:3, 3] = tvec.reshape(3,)
        return T
    
    def invert_transform(self,T):
        Rot = T[:3,:3]
        t = T[:3,3]
        Rinv = Rot.T
        tinv = -Rinv @ t
        Tinv = np.eye(4, dtype=np.float64)
        Tinv[:3,:3] = Rinv
        Tinv[:3,3] = tinv
        return Tinv
    
    def estimatePoseSingleMarker(self, corners, marker_size, camera_matrix, distortion_coefficients):
        distortion_matrix=distortion_coefficients
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners: # c is 4 corners in pixels for each marker
            nada, R, t = cv2.solvePnP(marker_points, c, camera_matrix, distortion_matrix)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return np.array(rvecs), np.array(tvecs), trash
    
    def matrix_to_pose(self, T: np.ndarray) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = float(T[0, 3])
        pose.pose.position.y = float(T[1, 3])
        pose.pose.position.z = float(T[2, 3])
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        rot = R.from_matrix(T[:3, :3])
        q = rot.as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        return pose




def main(args=None):
    rclpy.init(args=args)
    node = GtTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
