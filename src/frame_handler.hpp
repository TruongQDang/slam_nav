#include "vo_utils.hpp"

class VisualOdometry
{
public:
        VisualOdometry();
        pair<cv::Mat, cv::Mat> run(Mat &input_image);
        Mat img1, img2;
        int frame_id = 0;
        Mat R_f, t_f;
        double focal;
        Point2d pp;
        vector<Point2f> points1, points2;
        Mat traj = Mat::zeros(600, 600, CV_8UC3);
        Mat prev_image;
        Mat curr_image;
        vector<Point2f> prev_features;
        vector<Point2f> curr_features;
private:
        void process_first_frame(Mat &input_image);
        void process_second_frame(Mat &input_image);
        void process_normal_frame(Mat &input_image);
        void trajectory_visualization();
};

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

VisualOdometry::VisualOdometry()
{
        get_calibration_parameters("/home/truongdang/Documents/visual_slam/kitti_dataset/data_odometry_gray/dataset/sequences/00/calib.txt", focal, pp);
        draw_ground_truth_trajectory(traj, "/home/truongdang/Documents/visual_slam/kitti_dataset/data_odometry_poses/dataset/poses/00.txt");
}

pair<cv::Mat, cv::Mat> VisualOdometry::run(Mat &input_image)
{   
        if (frame_id == 0) {
                process_first_frame(input_image);
                Mat R = (cv::Mat_<double>(3, 3) << 
                        1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1);
                Mat t = (cv::Mat_<double>(3, 1) << 
                        0, 0, 0);
                return {R, t};
        } else if (frame_id == 1) {
                process_second_frame(input_image);
                return {R_f, t_f};
        } else {
                process_normal_frame(input_image);
                trajectory_visualization();
                return {R_f, t_f};
        }
}

void VisualOdometry::process_first_frame(Mat &input_image)
{
        img1 = input_image.clone();
        // feature detection
        feature_detection(img1, points1);
        frame_id++;
        cout << "Done first image" << endl;
}

void VisualOdometry::process_second_frame(Mat &input_image)
{
        img2 = input_image;
        if (!img1.data || !img2.data)
        {
                cout << "Error reading images! " << endl;
                return;
        }
        // feature tracking
        vector<uchar> status;
        feature_tracking(img1, img2, points1, points2, status);
        // // recover essential matrix and camera pose
        Mat E, R, t, mask;
        E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, points2, points1, R, t, focal, pp, mask);
        prev_image = img2.clone();
        prev_features = points2;

        R_f = R.clone();
        t_f = t.clone();
        frame_id++;
        cout << "Done second image" << endl;
}

void VisualOdometry::process_normal_frame(Mat &input_image)
{
        curr_image = input_image;
        vector<uchar> status;  
        feature_tracking(prev_image, curr_image, prev_features, curr_features, status);

        Mat E, R, t , mask;
        E = findEssentialMat(curr_features, prev_features, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, curr_features, prev_features, R, t, focal, pp, mask);

        t_f = t_f + R_f * t;
        R_f = R * R_f;

        // feature detection retrigggered if number of features below threshold
        if (prev_features.size() < MIN_NUM_FEAT) {
                feature_detection(prev_image, prev_features);
                feature_tracking(prev_image, curr_image, prev_features, curr_features, status);
        }

        prev_image = curr_image.clone();
        prev_features = curr_features;
        frame_id++;
        cout << "Processing normal frame " << frame_id << endl;
}

void VisualOdometry::trajectory_visualization()
{
        int x = int(t_f.at<double>(0)) + 300;      // offset for easier visualisation
        int y = int(-1 * t_f.at<double>(2)) + 500; // -1 inversion and offset for easier visualisation
        circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        imshow("Road facing Egomotion camera", curr_image);
        imshow("Trajectory", traj);
        waitKey(1);
}
