#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include "../3rd_party_libs/opencv3.4/include/opencv2/core/utility.hpp"
#define DRAW_AXES 0

using namespace cv;
using namespace std;

tuple<double, double> horizontalVerticalOffset(string image_path, string calibration_params) {
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs(calibration_params, cv::FileStorage::READ); // Read calibration parameters from file storage
    
    fs["camera_matrix"] >> cameraMatrix; // Read camera matrix
    fs["distortion_coefficients"] >> distCoeffs; // Read distortion coefficients
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); // Get a predefined dictionary DICT_6X6_250 containing a set of ArUco markers
    
    string image_file = samples::findFile(image_path);
    Mat image = imread(image_file, IMREAD_COLOR); // Retrieve image
    Mat imageCopy;
    
    image.copyTo(imageCopy);
    vector<int> ids;
    vector<vector<Point2f>> corners;
    detectMarkers(image, dictionary, corners, ids); // ArUco marker detection
    
    if (ids.size() > 0) {
        // at least one marker detected
        aruco::drawDetectedMarkers(imageCopy, corners, ids); // Draw detected markers in image. Can be further used for debugging purposes
        vector<cv::Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, 0.0975, cameraMatrix, distCoeffs, rvecs, tvecs); // Pose estimation for single markers
        Vec3d rvec = rvecs[0]; // first element of the array of output rotation vectors of detected markers. Each element in rvecs corresponds to the specific marker
        Vec3d tvec = tvecs[0]; // first element of the array of output translation vectors of detected markers. Each element in tvecs corresponds to the specific marker
        
        // Source: https://stackoverflow.com/questions/51476702/pose-of-camera-from-the-pose-of-marker
        Mat R;
        cv::Rodrigues(rvec, R); // Calculate marker pose R matrix
        Mat camR = R.t();  // Calculate camera R matrix
        Mat camRvec;
        Rodrigues(R, camRvec); // Calculate camera rvec
        Mat camTvec = -camR * tvec; // Calculate camera translation vector
        Vec3d camTvecArr((double*)camTvec.data); // Convert to vector
        
        double vertical_offset = camTvecArr[2]; // Calculate vertical offset
        double horizontal_offset = sqrt(pow(camTvecArr(0), 2) + pow(camTvecArr(1), 2)); // Calculate horizontal offset using pythagorean theorem
        
#if DRAW_AXES
        // Axes representing the marker's coordinate system can visualized for debugging purposes
        aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1); // Draw axes of the detected marker
        imshow("", imageCopy);
        int k = waitKey(0);
        if(k == 's') {
            imwrite(image_path, imageCopy);
        }
#endif // DRAW_AXES
        
        return std::make_tuple(horizontal_offset, vertical_offset); // Return a tuple of (horizontal_offset, vertical_offset)
    }
    
    return  std::make_tuple(-1.0, -1.0); // Return (-1.0, -1.0) in case estimation failed
}

int main(int argc, char *argv[]) {
    double horizontal_offset, vertical_offset;
    
    string calibration_params = "./input/calibration_params.yml"; // Calibration parameters file path
    string input = "./input/marker.jpg"; // Marker image file input path
    tie(horizontal_offset, vertical_offset) = horizontalVerticalOffset(input, calibration_params);
    cout.precision(8);
    cout << "Horizontal and Vertical Offsets: " << fixed << horizontal_offset << ';' << vertical_offset << endl;
}

