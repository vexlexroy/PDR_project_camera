import cv2
import numpy as np
import argparse
import time

def main():
    parser = argparse.ArgumentParser(description='Live camera calibration using checkerboard.')
    parser.add_argument('--rows', type=int, required=True, help='Number of internal corners per row')
    parser.add_argument('--cols', type=int, required=True, help='Number of internal corners per column')
    parser.add_argument('--size', type=float, required=True, help='Checkerboard square in mm')
    parser.add_argument('--num', type=int, default=15, help='Number of images to capture')
    parser.add_argument('--interval', type=int, default=10, help='Capture interval')
    parser.add_argument('--id', type=int, default=0, help='Camera device ID')
    args = parser.parse_args()

    CHECKERBOARD = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= args.size

    objpoints = []
    imgpoints = []

    cap = cv2.VideoCapture(args.id)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to quit early.")
    print("Searching for checkerboard...")

    frame_count = 0
    captured = 0

    while captured < args.num:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_cb, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display_frame = frame.copy()
        if ret_cb:
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_cb)

        cv2.putText(display_frame, f"Captured: {captured}/{args.num}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if ret_cb else (0, 0, 255), 2)
        cv2.imshow('Calibration', display_frame)

        if ret_cb and frame_count % args.interval == 0:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            captured += 1
            print(f"Captured image {captured}/{args.num}")
            time.sleep(0.1)  # short pause to avoid duplicates

        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if captured < 3:
        print("Not enough valid images captured for calibration.")
        return

    print("Calibrating camera...")

    flags = (
    cv2.CALIB_FIX_K3 |
    cv2.CALIB_ZERO_TANGENT_DIST
    )

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=flags)
    if not ret:
        print("Calibration failed.")
        return

    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]

    print("\n=== Calibration Results ===")
    print("Camera matrix:\n", mtx)
    print("\nDistortion coefficients:\n", dist.ravel())
    print(f"\n[fx, cx, fy, cy] = [{fx:.3f}, {cx:.3f}, {fy:.3f}, {cy:.3f}]")

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], mtx, dist
        )
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print("Reprojection error:", mean_error / len(objpoints))

if __name__ == '__main__':
    main()
