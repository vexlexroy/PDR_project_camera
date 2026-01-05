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
        self.testing = self.get_parameter('testing').value
        self.declare_parameter('max_path', 5000)
        self.max_path = self.get_parameter('max_path').value

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

        self.world_marker_transforms={}

    def proces_images(self,image_msg:Image): 
        #undistort image
        frame= self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        undistorted_image = cv2.undistort(
            frame,
            self.camera_matrix,
            self.distortion_matrix
        )
        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        #detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(gray,corners,ids)
        rvecs, tvecs, _ = self.estimatePoseSingleMarker(
                    corners, self.marker_size_mm/1000, self.camera_matrix, np.zeros(5,1) #self.distortion_matrix #zero if undistorted image
                )
        #check for known markers
        if(ids):
            if(len(self.world_marker_transforms)==0): #initial marker memory
                for i,x in enumerate(ids):
                    rvec=rvecs[i]
                    tvec=tvecs[i]
                    T_cam_marker=self.make_transform(rvec, tvec)
                    self.world_marker_transforms[x] = T_cam_marker # camera starts as world origin
                #sets base pose and publishes it
                self.last_cam_pose=PoseStamped()
                self.last_cam_pose.pose.position.x=0.0
                self.last_cam_pose.pose.position.y=0.0
                self.last_cam_pose.pose.position.z=0.0
                self.last_cam_pose.pose.orientation.x=0.0
                self.last_cam_pose.pose.orientation.y=0.0
                self.last_cam_pose.pose.orientation.z=0.0
                self.last_cam_pose.pose.orientation.w=1.0
                self.last_cam_pose.header.stamp = self.get_clock().now().to_msg()
                self.last_cam_pose.header.frame_id = 'cam'
                self.camera_pose.publish(self.last_cam_pose)
                #adds begining to path
                pose_for_path = PoseStamped()
                pose_for_path.header = self.last_cam_pose.header
                pose_for_path.pose = self.last_cam_pose.pose
                self.camera_path.header.stamp = self.get_clock().now().to_msg()
                self.camera_path.poses.append(pose_for_path)
                self.path_pub.publish(self.camera_path) 
            else:
                #check if known marker exists
                T_w_m=None # world marker transform
                T_c_m=None # camera marker transform
                T_w_c=None # world camera trransform
                for i,x in enumerate(ids):
                    if(x in self.world_marker_transforms):
                        T_w_m = self.world_marker_transforms[x]
                        rvec = rvecs[i]
                        tvec = tvecs[i]
                        T_c_m = self.make_transform(rvec, tvec)
                        break
                #calculate cmera world pose based on known marker
                if(T_w_m):
                    T_w_c = T_w_m @ self.invert_transform(T_c_m)
                    #calculate world marker pose for other new markers
                    for i,x in enumerate(ids):
                        if(x not in self.world_marker_transforms):
                            rvec = rvecs[i]
                            tvec = tvecs[i]
                            T_c_nm = self.make_transform(rvec, tvec) # camera new marker transform
                            T_w_nm = T_w_c @ T_c_nm # world new marker transform
                            self.world_marker_transforms[x]=T_w_nm
                    #publish new pose of camera
                    pose = self.matrix_to_pose(T_w_c)
                    self.last_cam_pose=pose
                    self.camera_pose.publish(pose)
                    
                    pose_for_path = PoseStamped()
                    pose_for_path.header = pose.header
                    pose_for_path.pose = pose.pose
                    self.camera_path.header.stamp = self.get_clock().now().to_msg()
                    self.camera_path.poses.append(pose_for_path)
                    if(len(self.camera_path.poses)>self.max_path):
                        self.camera_path.poses.pop(0)
                    self.path_pub.publish(self.camera_path)
                
                else: # skip any action if no marker corilation was found
                    return
        #publish image of detected markers
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
        pose.header.frame_id = 'cam'
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
