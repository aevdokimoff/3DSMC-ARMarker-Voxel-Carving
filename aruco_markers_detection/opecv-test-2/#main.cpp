#include<opencv2/opencv.hpp>
#include<iostream>


using namespace cv;
using namespace std;

#define EX5 1
#define DRAW_CONTOUR 0
#define DRAW_RECTANGLE 0

#define THICKNESS_VALUE 4


// List of points
typedef vector<Point> contour_t;
// List of contours
typedef vector<contour_t> contour_vector_t;


const int threshold_slider_max = 255;
int threshold_slider = 0;

const int fps = 30;

Mat videoStreamFrameGray;
Mat videoStreamFrameOutput;

// Pos is from UI, dereferencing of the pointer
static void on_trackbar(int pos, void* slider_value) {
    *((int*)slider_value) = pos;
    // C++ >= 11 -> Standard
    //*(static_cast<int*>(slider_value)) = pos;

    // E.g. all values from 170 to 255 are black, the rest under 170 is white. Thresholding takes the image as a matrix.
    //threshold(videoStreamFrameGray, videoStreamFrameOutput, threshold_slider, threshold_slider_max, THRESH_BINARY);
}


#if EX5

    int main() {

    Mat frame;
    VideoCapture cap(1);

    if (!cap.isOpened()) {
        cout << "No webcam, using video file" << endl;
        cap.open("/Users/artem/Projects/studium-tum-projects/3dsmc/final-project/opecv-test-2/MarkerMovie.mp4");
        if (cap.isOpened() == false) {
            cout << "No video!" << endl;
            exit(0);
        }
    }

    const string contoursWindow = "Contours";
    const string UI = "Threshold";
    namedWindow(contoursWindow, CV_WINDOW_FREERATIO);

    int slider_value = 80;
    createTrackbar(UI, contoursWindow, &slider_value, 255, on_trackbar, &slider_value);

    Mat imgFiltered;

    while (cap.read(frame)) {

        // --- Process Frame ---

        Mat grayScale;
        imgFiltered = frame.clone();
        cvtColor(imgFiltered, grayScale, COLOR_BGR2GRAY);

        // Threshold to reduce the noise
        if (slider_value == 0) {
            adaptiveThreshold(grayScale, grayScale, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);
        }
        else {
            threshold(grayScale, grayScale, slider_value, 255, THRESH_BINARY);
        }

        contour_vector_t contours;

        // RETR_LIST is a list of all found contour, SIMPLE is to just save the begin and ending of each edge which belongs to the contour
        findContours(grayScale, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        //drawContours(imgFiltered, contours, -1, Scalar(0, 255, 0), 4);

        // size is always positive, so unsigned int -> size_t; if you have not initialized the vector it is -1, hence crash
        for (size_t k = 0; k < contours.size(); k++) {

            // -------------------------------------------------

            // --- Process Contour ---

            contour_t approx_contour;

            // Simplifying of the contour with the Ramer-Douglas-Peuker Algorithm
            // true -> Only closed contours
            // Approximation of old curve, the difference (epsilon) should not be bigger than: perimeter(->arcLength)*0.02
            approxPolyDP(contours[k], approx_contour, arcLength(contours[k], true) * 0.02, true);

#if DRAW_CONTOUR
            contour_vector_t cov,aprox;
            cov.emplace_back(contours[k]);
            aprox.emplace_back(approx_contour);
            if (approx_contour.size() > 1) {
                drawContours(imgFiltered, cov, -1, Scalar(0, 255, 0), 4, 1);
                drawContours(imgFiltered, aprox, -1, Scalar(255, 0, 0), 4, 1);
                continue;
            }
#endif // DRAW_CONTOUR

            Scalar QUADRILATERAL_COLOR(0, 0, 255);
            Scalar colour;
            // Convert to a usable rectangle
            Rect r = boundingRect(approx_contour);

#if DRAW_RECTANGLE
            rectangle(imgFiltered, r, QUADRILATERAL_COLOR, 4);
            continue;
#endif //DRAW_RECTANGLE

            // 4 Corners -> We color them
            if (approx_contour.size() == 4) {
                colour = QUADRILATERAL_COLOR;
            }
            else {
                continue;
            }

            // --- Filter tiny ones --- If the found contour is too small (20 -> pixels, frame.cols - 10 to prevent extreme big contours)
            if (r.height < 20 || r.width < 20 || r.width > imgFiltered.cols - 10 || r.height > imgFiltered.rows - 10) {
                continue;
            }

            // -> Cleaning done!

            // 1 -> 1 contour, we have a closed contour, true -> closed, 4 -> thickness
            polylines(imgFiltered, approx_contour, true, colour, THICKNESS_VALUE);
        
            // -----------------------------

            // --- Process Corners ---

            for (size_t i = 0; i < approx_contour.size(); ++i) {
                // Render the corners, 3 -> Radius, -1 filled circle
                circle(imgFiltered, approx_contour[i], 3, CV_RGB(0, 255, 0), -1);

                // Euclidic distance, 7 -> parts, both directions dx and dy
                double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
                double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;

                // First point already rendered, now the other 6 points
                for (int j = 1; j < 7; ++j) {
                    // Position calculation
                    double px = (double)approx_contour[i].x + (double)j * dx;
                    double py = (double)approx_contour[i].y + (double)j * dy;

                    Point p;
                    p.x = (int)px;
                    p.y = (int)py;
                    circle(imgFiltered, p, 2, CV_RGB(0, 0, 255), -1);
                }
            }
            
            // -----------------------------

            // -------------------------------------------------
        }

        imshow(contoursWindow, imgFiltered);

        if (waitKey(10) == 27) {
            break;
        }
    }

    destroyWindow(contoursWindow);
#else

    const string webcamStream = "Captured Webcam Stream";
    namedWindow(webcamStream, CV_WINDOW_NORMAL);

    while (cap.read(frame)) {
        cvtColor(frame, videoStreamFrameGray, CV_BGR2GRAY);

        char TrackbarName[50];
        // Format data to String
        sprintf_s(TrackbarName, "Threshold x %d", threshold_slider_max);

        // Mat _ = Value
        // Mat* = Pointer = Adress
        // Mat& = Reference of a Matrix = Adress in memory representative for the Value, an alias
        createTrackbar(TrackbarName, webcamStream, &threshold_slider, threshold_slider_max, on_trackbar, &threshold_slider);

        if (threshold_slider == 0) {
            adaptiveThreshold(videoStreamFrameGray, videoStreamFrameOutput, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);
        }
        else {
            threshold(videoStreamFrameGray, videoStreamFrameOutput, threshold_slider, threshold_slider_max, THRESH_BINARY);
        }

        imshow(webcamStream, videoStreamFrameOutput);

        // FPS
        int key = waitKey(10);
        if (key == 27) {
            break;
        }
    }

    destroyWindow(webcamStream);
#endif

    return(0);
}



