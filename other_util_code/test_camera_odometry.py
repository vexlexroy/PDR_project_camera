import cv2
import numpy as np


def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def get_transform(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    # Create homogeneous transform
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = tvec.reshape(3)
    return T

def matrix_to_pose(T: np.ndarray):
    x = float(T[0, 3])
    y = float(T[1, 3])
    z = float(T[2, 3])
    return x,y,z


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(dictionary=aruco_dict,detectorParams=parameters)

    id = 2
    camera_matrix= np.array(([523.56400802,   0.,         299.76402461],
                    [  0.,         523.91329778, 241.14238425],
                    [  0.,           0.,           1.        ]))
    distortion_matrix= np.array([-0.09235926,  0.14059952,  0.,          0.,         -0.01142402])
    marker_size_mm=151
    world_marker_transforms={}
    
    #open camera
    cap = cv2.VideoCapture(id)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    print("Press 'q' to quit early.")

    # get new frame
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        #undistort image
        h, w = frame.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            distortion_matrix,
            (w, h),
            0.0,
            (w, h)
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            distortion_matrix,
            None,
            newK,
            (w, h),
            cv2.CV_16SC2
        )
        undistorted_image = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        # detect markers
        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(gray,corners,ids)
        cv2.imshow('display',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # compute 3d pose
        rvecs, tvecs, _ = estimatePoseSingleMarkers(
                    corners, marker_size_mm/1000, newK, np.zeros(5)
                )
        #get all transform
        if(ids is not None and len(ids) > 0):
            if(len(world_marker_transforms)==0): #initial world setup
                for i,x in enumerate(ids):
                    world_marker_transforms[int(x)]=get_transform(rvecs[i],tvecs[i])
            else:
                T_w_m_id=[]
                T_c_m=None
                T_w_c=[]
                for i,x in enumerate(ids):
                    if(int(x) in world_marker_transforms):# check if known marker exists
                        T_w_m_id.append(int(x))
                if(len(T_w_m_id)!=0):
                    for i,x in enumerate(ids):
                        T_c_m = get_transform(rvecs[i],tvecs[i]) # camera marker transform
                        if(int(x) in T_w_m_id):
                            T_w_m=world_marker_transforms[int(x)]
                            T_w_c.append(np.linalg.inv(T_c_m)@T_w_m) # get world camera transform
                    for i,x in enumerate(ids):
                        T_c_mn = get_transform(rvecs[i],tvecs[i]) # camera marker transform
                        if(int(x) not in T_w_m_id):
                            world_marker_transforms[int(x)]=T_w_c[0]@T_c_mn
                    x,y,z=matrix_to_pose(T_w_c[0])
                    print(f"x: {x:.4f} y: {y:.4f} z: {z:.4f}")








if __name__ == '__main__':
    main()