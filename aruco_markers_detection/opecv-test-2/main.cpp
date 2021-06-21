#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main() {
    
    VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        cout << "No webcam, using video file" << endl;
        // change it to your path
        cap.open("/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/aruco_markers_detection/opecv-test-2/MarkerMovie.mp4");
        if (cap.isOpened() == false) {
            cout << "No video!" << endl;
            exit(0);
        }
    }
    
    cv::Mat cameraMatrix, distCoeffs;
    // change it to your path
    cv::FileStorage fs("/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/aruco_markers_detection/opecv-test-2/calibration_params.yml", cv::FileStorage::READ);
    
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
    std::cout << "camera_matrix\n" << cameraMatrix << std::endl;
    std::cout << "\ndist coeffs\n" << distCoeffs << std::endl;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    while (cap.grab()) {
        cv::Mat frame, frameCopy;
        cap.retrieve(frame);
        frame.copyTo(frameCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs; // rotation and translation vectors -> camera pose
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i<ids.size(); i++)
                cv::aruco::drawAxis(frameCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
        cv::imshow("out", frameCopy);
        int key = waitKey(10);
        if (key == 27) {
            break;
        }
    }
    
}
