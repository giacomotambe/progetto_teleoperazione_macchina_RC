import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm

class VisualOdometry:
    def __init__(self, video_path, data_dir):
        self.K, self.P = self._load_calib(os.path.join(data_dir, 'calib.txt'))
        #self.gt_poses = self._load_poses(os.path.join(data_dir,"poses.txt"))
        self.video_path = video_path

        # Capture the video
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise ValueError("Error opening video file: " + video_path)
        
        self.orb = cv2.ORB_create(3000)
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

    @staticmethod
    def _load_calib(filepath):
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P

    @staticmethod
    def _load_poses(filepath):
        poses = []
        with open(filepath, 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, dtype=np.float64, sep=' ')
                T = T.reshape(3, 4)
                T = np.vstack((T, [0, 0, 0, 1]))
                poses.append(T)
        return poses

    @staticmethod
    def _load_images(filepath):
        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]
        return [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]

    @staticmethod
    def _form_transf(R, t):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def get_matches(self, i, scale):
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, scale*(i-1))
        ret1, frame1 = self.cap.read()
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, scale*i)
        ret2, frame2 = self.cap.read()

        if not ret1 or not ret2:
            raise ValueError("Could not read frames from video.")

        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        kp1, des1 = self.orb.detectAndCompute(gray1, None)
        kp2, des2 = self.orb.detectAndCompute(gray2, None)

        matches = self.flann.knnMatch(des1, des2, k=2)

        good = []
        try:
            for m, n in matches:
                if m.distance < 0.8 * n.distance:
                    good.append(m)
        except ValueError:
            pass

        draw_params = dict(matchColor=-1,  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=None,  # draw only inliers
                           flags=2)
        img3 = cv2.drawMatches(gray1, kp1, gray2, kp2, good, None, **draw_params)
        cv2.imshow("image", img3)
        cv2.waitKey(200)

        q1 = np.float32([kp1[m.queryIdx].pt for m in good])
        q2 = np.float32([kp2[m.trainIdx].pt for m in good])
        return q1, q2

    def get_pose(self, q1, q2):
        E, _ = cv2.findEssentialMat(q1, q2, self.K, threshold=1)
        R, t = self.decomp_essential_mat(E, q1, q2)
        transformation_matrix = self._form_transf(R, np.squeeze(t))
        return transformation_matrix

    def decomp_essential_mat(self, E, q1, q2):
        def sum_z_cal_relative_scale(R, t):
            T = self._form_transf(R, t)
            P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)

            hom_Q1 = cv2.triangulatePoints(self.P, P, q1.T, q2.T)
            hom_Q2 = np.matmul(T, hom_Q1)

            uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]

            sum_of_pos_z_Q1 = sum(uhom_Q1[2, :] > 0)
            sum_of_pos_z_Q2 = sum(uhom_Q2[2, :] > 0)

            relative_scale = np.mean(np.linalg.norm(uhom_Q1.T[:-1] - uhom_Q1.T[1:], axis=-1) /
                                     np.linalg.norm(uhom_Q2.T[:-1] - uhom_Q2.T[1:], axis=-1))
            return sum_of_pos_z_Q1 + sum_of_pos_z_Q2, relative_scale

        R1, R2, t = cv2.decomposeEssentialMat(E)
        t = np.squeeze(t)

        pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]

        z_sums = []
        relative_scales = []
        for R, t in pairs:
            z_sum, scale = sum_z_cal_relative_scale(R, t)
            z_sums.append(z_sum)
            relative_scales.append(scale)

        right_pair_idx = np.argmax(z_sums)
        right_pair = pairs[right_pair_idx]
        relative_scale = relative_scales[right_pair_idx]
        R1, t = right_pair
        t = t * relative_scale

        return [R1, t]


def main():
    data_dir = "./"  # Try KITTI_sequence_2 too
    vo = VisualOdometry("odometry2.avi", data_dir)

    estimated_path = []

    # Initialize the plot
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("Camera Trajectory")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Z Position")
    line, = ax.plot([], [], 'b-')  # Line object to update

    tot_frames = int(vo.cap.get(cv2.CAP_PROP_FRAME_COUNT))-40

    scale=10

    for i in tqdm(range(tot_frames//scale)):

            if i == 0:
                cur_pose = np.array([
                        [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0]])
            else:
                q1, q2 = vo.get_matches(i, scale)
                transf = vo.get_pose(q1, q2)
                cur_pose = np.matmul(cur_pose, np.linalg.inv(transf))
        
            # Update the estimated path
            estimated_path.append((cur_pose[0, 3], cur_pose[2, 3]))

            # Update plot data
            x_data, z_data = zip(*estimated_path)
            line.set_xdata(x_data)
            line.set_ydata(z_data)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)  # Pause to update the plot

    plt.ioff()  # Turn off interactive plotting
    plt.show()  # Show the final plot


if __name__ == "__main__":
    main()
