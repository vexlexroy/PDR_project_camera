import heapq
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


def autoscale_with_min(ax, xmin, xmax, ymin, ymax):
    ax.relim()
    ax.autoscale_view()

    # get current limits
    cur_xmin, cur_xmax = ax.get_xlim()
    cur_ymin, cur_ymax = ax.get_ylim()

    # enforce minimum range
    cur_xmin = min(cur_xmin, xmin)
    cur_xmax = max(cur_xmax, xmax)
    cur_ymin = min(cur_ymin, ymin)
    cur_ymax = max(cur_ymax, ymax)

    ax.set_xlim(cur_xmin, cur_xmax)
    ax.set_ylim(cur_ymin, cur_ymax)

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
    rot = R.from_matrix(T[:3, :3])
    q = rot.as_quat()  # [x, y, z, w]
    ang=rot.as_euler('xyz', degrees=True)


    return x,y,z,q,ang


def main():
    x_path=[]
    y_path=[]
    z_path=[]
    q_path=[]
    ang_path=[]
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(dictionary=aruco_dict,detectorParams=parameters)

    id = 2
    camera_matrix = np.array([
        [528.963795, 0.0,       300.441205],
        [0.0,        529.372856, 241.815110],
        [0.0,        0.0,         1.0]
    ], dtype=np.float32)

    distortion_matrix = np.array([
        -0.082430,
        0.152254,
        -0.003181,
        -0.001335,
        0.000000
    ], dtype=np.float32)
    marker_size_mm=151
    world_marker_transforms={}
    T_ros_world = np.array([
                        [ 0,  0,  1, 0],
                        [ 1,  0,  0, 0],
                        [ 0, -1,  0, 0],
                        [ 0,  0,  0, 1]])
    

    #ploting
    plt.ion()
    fig, ax = plt.subplots()
    sc = ax.scatter([], [], c=[], cmap='viridis', s=15)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    plt.colorbar(sc, ax=ax, label="Z [m]")
    ax.set_title("Live camera position")
    ax.axis("equal")
    ax.grid(True)

    #open camera
    cap = cv2.VideoCapture(id)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    # get new frame
    frame_counter=0
    pose_on=5
    last_pose=None
    min_motion=0.05
    mapping_enabled = False
    print("Press 'm' to ENABLE mapping, 'l' to LOCK mapping, 'q' to quit")
    while True:
        frame_counter=frame_counter+1
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
        if cv2.waitKey(1) & 0xFF == ord('m'):
            mapping_enabled=True
            print("mapping on")
        if cv2.waitKey(1) & 0xFF == ord('l'):
            mapping_enabled=False
            print("mapping off")

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
                T_w_m_id=set()
                T_c_m=None
                T_w_c=[]
                for i,x in enumerate(ids):
                    if(int(x) in world_marker_transforms):# check if known marker exists
                        T_w_m_id.add(int(x))
                if(len(T_w_m_id)!=0): # if there are known markers
                    for i,x in enumerate(ids):
                        T_c_m = get_transform(rvecs[i],tvecs[i]) # camera marker transform
                        if(int(x) in T_w_m_id): #if marker is known
                            T_w_m=world_marker_transforms[int(x)] # load world to marker T
                            T_w_c_mat=T_w_m@np.linalg.inv(T_c_m) # calculate world camera transform
                            grade=np.linalg.norm(T_c_m[:3,3]) # calculate how close to camera is it
                            heapq.heappush(T_w_c, (grade, T_w_c_mat)) # aprased world camera transform by closeness
                    # T_w_c.sort(key=lambda x: x[1]) # sort by closest marker
                    best_T_w_c=T_w_c[0][1]
                    if(mapping_enabled): # if maping enabled
                        for i,x in enumerate(ids): # calculate other marker positions
                            T_c_mn = get_transform(rvecs[i],tvecs[i]) # camera marker transform for new marker
                            if(int(x) not in T_w_m_id): # if it doesent already exist
                                world_marker_transforms[int(x)]=best_T_w_c@T_c_mn # make noew world marker transform based on known world camera transform
                    if(frame_counter%pose_on==0): # update pose every n frames / ploting
                        if(last_pose is not None and np.linalg.norm(last_pose-best_T_w_c)>min_motion): # check how similar is last pose
                            last_pose=best_T_w_c
                            x,y,z,q,ang=matrix_to_pose(T_ros_world@best_T_w_c) # get data from matrix rotated to ros system x->forward, y->right, z->up
                            x_path.append(x)
                            y_path.append(y)
                            z_path.append(z)
                            q_path.append(q)
                            ang_path.append(ang)
                        else:
                            last_pose=best_T_w_c

                        
                        sc.set_offsets(np.column_stack((x_path, y_path)))
                        sc.set_array(np.array(z_path))   # Z controls color

                        autoscale_with_min(ax, -1, 1, -1, 1)
                        plt.pause(0.001)
                    # print(f"x: {x:.4f} y: {y:.4f} z: {z:.4f}")


    plt.show()





if __name__ == '__main__':
    main()