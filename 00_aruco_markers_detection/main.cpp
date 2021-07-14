#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include "../3rd_party_libs/opencv3.4/include/opencv2/core/utility.hpp"

using namespace cv;
using namespace std;

tuple<double, double> horizontalVerticalOffset(string image_path, string calibration_params) {
    cv::Mat cameraMatrix, distCoeffs;
    // change it to your path
    cv::FileStorage fs(calibration_params, cv::FileStorage::READ);
    
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    string image_file = samples::findFile(image_path);
    Mat image = imread(image_file, IMREAD_COLOR);
    Mat imageCopy;
    
    image.copyTo(imageCopy);
    vector<int> ids;
    vector<vector<Point2f>> corners;
    detectMarkers(image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        vector<cv::Vec3d> rvecs, tvecs; // rotation and translation vectors -> camera pose
        aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        Vec3d rvec = rvecs[0];
        Vec3d tvec = tvecs[0];
        // draw axis for the first detected marker
        aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
        double vertical_offset = tvec[2];
        double horizontal_offset = sqrt(pow(tvec(0), 2) + pow(tvec(1), 2));
        
        imshow("Display window", imageCopy);
        int k = waitKey(0);
        if(k == 's') {
            imwrite(image_path, imageCopy);
        }
        
        return  std::make_tuple(horizontal_offset, vertical_offset);
    }
    
    return  std::make_tuple(-1.0, -1.0);
}

int main() {
    string calibration_params = "/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/00_aruco_markers_detection/aruco-markers-detection/calibration_params.yml";
    string image_path = "/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/00_aruco_markers_detection/aruco-markers-detection/markers_side.jpg";
    double horizontal_offset, vertical_offset;
    tie(horizontal_offset, vertical_offset) = horizontalVerticalOffset(image_path, calibration_params);
    cout << horizontal_offset << ',' << vertical_offset << endl;
}

