#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main() {
    
    cv::Mat cameraMatrix, distCoeffs;
    // change it to your path
    cv::FileStorage fs("/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/aruco_markers_detection/opecv-test-2/calibration_params.yml", cv::FileStorage::READ);
    
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
    std::cout << "camera_matrix\n" << cameraMatrix << std::endl;
    std::cout << "\ndist coeffs\n" << distCoeffs << std::endl;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    // change it to your path
    string image_path = "/Users/artem/Projects/3DSMC-ARMarker-Voxel-Carving/aruco_markers_detection/opecv-test-2/markers_side.jpg";
    string image_file = samples::findFile(image_path);
    Mat image = imread(image_file, IMREAD_COLOR);
    Mat imageCopy;
    
    image.copyTo(imageCopy);
    vector<int> ids;
    vector<std::vector<cv::Point2f>> corners;
    aruco::detectMarkers(image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        vector<cv::Vec3d> rvecs, tvecs; // rotation and translation vectors -> camera pose
        aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        // draw axis for each marker
        for(int i=0; i<ids.size(); i++)
            aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    }
    imshow("Display window", imageCopy);
    int k = waitKey(0);
    if(k == 's') {
        imwrite(image_path, imageCopy);
    }
    
}
