#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace cv;
using namespace std;

void feature_detection(Mat img, vector<Point2f> &points)
{
        int fast_threshold = 20;
        bool nonmaxSuppression = true;
        vector<KeyPoint> keypoints;
        FAST(img, keypoints, fast_threshold, nonmaxSuppression);
        KeyPoint::convert(keypoints, points, vector<int>());
}

void feature_tracking(Mat img1, Mat img2, vector<Point2f> &points1, vector<Point2f> &points2, vector<uchar> &status)
{       
        vector<float> err;
        calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err);
        
        // remove points for which the KLT tracking failed or those who have gone outside the frame
        int index_correction = 0;
        for (unsigned int i = 0; i < status.size(); i++) {
                Point2f pt = points2.at(i - index_correction);
                if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
                        if ((pt.x < 0) || (pt.y < 0)) {
                                status.at(i) = 0;
                        }
                        points1.erase(points1.begin() + (i - index_correction));
                        points2.erase(points2.begin() + (i - index_correction));
                        index_correction++;
                }
        }

}

void get_calibration_parameters(string calibration_file_path, double &focal, Point2d &pp)
{
        ifstream calibration_file(calibration_file_path);

        if (!calibration_file.is_open())
        {
                cout << "Error: Calibration file not found!" << endl;
                return;
        }

        string line;
        getline(calibration_file, line);
        istringstream iss(line);
        vector<string> calibration_parameters((istream_iterator<string>(iss)), istream_iterator<string>());
        focal = stod(calibration_parameters[1]);
        pp.x = stod(calibration_parameters[3]);
        pp.y = stod(calibration_parameters[7]);

        calibration_file.close();
}

void draw_ground_truth_trajectory(Mat &traj, const std::string &pose_file)
{
  ifstream pose_stream(pose_file);
  string line;
  double x, z;
  int i = 0;

  if (pose_stream.is_open())
  {
    while (getline(pose_stream, line))
    {
      std::istringstream in(line);
      for (int j = 0; j < 12; j++)
      {
        in >> z;
        if (j == 3)
          x = z;
      }

      int x_plot = int(x) + 300;
      int y_plot = int(-z) + 500;
      circle(traj, Point(x_plot, y_plot), 1, CV_RGB(0, 255, 0), 2);
      i++;
    }
    pose_stream.close();
  }
  else
  {
    cout << "Unable to open pose file" << endl;
  }
}