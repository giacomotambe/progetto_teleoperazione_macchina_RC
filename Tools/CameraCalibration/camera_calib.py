#!/usr/bin/python3
import cv2 as cv
import numpy as np
import os

#-------TEORIA-------#
# FLAG #
'''
CALIB_CB_ADAPTIVE_THRESH: use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness)
CALIB_CB_NORMALIZE_IMAGE: normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding
CALIB_CB_FILTER_QUADS: use additional criteria (like contourn area, perimeter, square-like shape)
CALIB_CB_FAST_CHECK: run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed
'''
# cornerSubPix #
'''

'''
#--------------------#

#---NOTA---#
'''
front camera: chessSize = (7, 5)
left, right, rear: chessSize = (5, 7)
'''

'''
I video registrati devo essere nella stessa cartella dello script
'''
#----------#

# Camera choice
camera_name = "front"

# Chessboard dimensions
chessSize = (5, 7)  # 5 squares horizontally, 7 vertically
square_size = 0.044  # 44mm = 0.044m

# Global variables for calibration
chess = []
stop_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

def processVideo(video_path, debug=False):
    global objpoints, imgpoints
    objp = np.zeros((chessSize[0]*chessSize[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessSize[0], 0:chessSize[1]].T.reshape(-1, 2) * square_size
    
    cap = cv.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error opening video")
        return

    total_frames = int(cap.get(cv.CAP_PROP_FRAME_COUNT))
    print(f"Total frames in video: {total_frames}")

    count = 0
    frame_interval = 5  # Analyze every 10th frame

    while count < total_frames:
        ret, img = cap.read()
        if not ret:
            print(f"Cannot read frame {count}")
            break
        
        if count % frame_interval == 0:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            
            # Try different configurations for findChessboardCorners
            for flags in [cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE,
                          cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FAST_CHECK,
                          cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FILTER_QUADS]:
                ret, corners = cv.findChessboardCorners(gray, chessSize, flags)
                if ret:
                    cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), stop_criteria)
                    chess.append(img)
                    print(f"Chessboard image acquired: {len(chess)}")
                    print(f"Found {corners.shape[0]} corners")
                    
                    cv.drawChessboardCorners(img, chessSize, corners, ret)
                    cv.imshow("calib", img)
                    cv.waitKey(1)
                    
                    objpoints.append(objp)
                    imgpoints.append(corners)
                    break
                else:
                    print(f"No chessboard detected in frame {count}")

        # Show progress
        cv.imshow("Video", img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        count += 1
        if count % 100 == 0:
            print(f"Processed {count} frames")

    cap.release()
    cv.destroyAllWindows()
    print(f"Total frames analyzed: {count}")
    print(f"Total images with chessboard detected: {len(chess)}")

    if len(objpoints) > 0 and len(imgpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        if ret:
            print(f"Calibration successful with error: {ret}")
            saveCalibrationResults(mtx, dist)
            #undistortFrames(video_path, mtx, dist)
        else:
            print("Calibration failed.")
    else:
        print("Not enough points for calibration.")

def saveCalibrationResults(mtx, dist):
    '''Save calibration results to a text file'''
    calibration_file = f"calibration_results_{camera_name}.txt" 
    try:
        with open(calibration_file, "w") as f:
            f.write("Camera Matrix:\n")
            f.write(np.array2string(mtx, separator=', '))
            f.write("\n\nDistortion Coefficients:\n")
            f.write(np.array2string(dist, separator=', '))
        print(f"Calibration results saved to: {calibration_file}")
    except Exception as e:
        print(f"Error saving file: {e}")
'''
def undistortFrames(video_path, mtx, dist, sample_interval=50):
    # Undistort frames and save them in 'before_undistortion' and 'after_undistortion' directories
    cap = cv.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error opening video for undistortion")
        return
    
    before_undistortion_dir = "before_undistortion"
    after_undistortion_dir = "after_undistortion"
    
    os.makedirs(before_undistortion_dir, exist_ok=True)
    os.makedirs(after_undistortion_dir, exist_ok=True)
    
    count = 0
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_count % sample_interval == 0:
            undistorted = cv.undistort(frame, mtx, dist)
            
            before_path = os.path.join(before_undistortion_dir, f"frame_before_undistortion_{count}.jpg")
            after_path = os.path.join(after_undistortion_dir, f"frame_after_undistortion_{count}.jpg")
            
            cv.imwrite(before_path, frame)
            cv.imwrite(after_path, undistorted)

            screen_size_h = frame.shape[0] 
            screen_size_w = frame.shape[1] 
            screen_size = (screen_size_w, screen_size_h)
            print(screen_size)
            
            count += 1

        frame_count += 1

    cap.release()
    print(f"Undistortion complete. {count} frames processed.")
'''
# Example usage
if __name__ == "__main__":

    video_path = "./output.avi"
    processVideo(video_path, debug=True)
    print(f"Images collected: {len(chess)}")






