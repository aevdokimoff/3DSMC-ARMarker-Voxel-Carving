#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main()
{
	ifstream fin("calibdata.txt"); /* in this file, image names are stored */
	ofstream fout("caliberation_result.txt");  /* file for solving the result */
	
	//read each frame, extract corners from each frame, find subpixel-accurate positions of the chessboard corners
	cout << "Starting extract corners………………" << endl;
	int image_count = 0;  /* number of images */
	Size image_size;  /* size of images */
	Size board_size = Size(6, 9);    /* corners of each row & column on checkerborad */
	vector<Point2f> image_points_buf;  
	vector<vector<Point2f>> image_points_seq; /* save cormers of each frame */
	string filename;
	int count = -1;//save number of corners
	while (getline(fin, filename))
	{
		image_count++;
		// checkout output
		cout << "image_count = " << image_count << endl;
		cout << "-->count = " << count << endl;
		Mat imageInput = imread(filename);
		if (image_count == 1)  //when reading the first image, save width and height of the image
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		/* extract corners */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"<< endl; //cannot find corners
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* finds subpixel-accurate positions of the chessboard corners */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(11, 11)); //accuarte the corners' location
			image_points_seq.push_back(image_points_buf);  //save sub-pixel
			/* show corners on image */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); 
			imshow("Camera Calibration", view_gray);
			waitKey(500);//wait for 0.5s
		}
	}
	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width * board_size.height;  //number of corners on each frame
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// in our cases, there are 54 corners on each image. print number of images to control the process 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			//cout << "--> The " << j << "-st image --> : " << endl;
		}
		if (0 == ii % 3)	
		{
			//cout << endl;
		}
		else
		{
			//cout.width(10);
		}
		//print out corners
		//cout << " -->" << image_points_seq[ii][0].x << endl; 
		//cout << " -->" << image_points_seq[ii][0].y << endl;
	}
	cout << "Corners extraction is Done！\n" ;

	//Camera Callibration
	cout << "\nStarting to calibration………………" << endl;
	/*Information of checkerboard*/
	Size square_size = Size(24, 24);  /* Size of checkerboard (real measured) */
	vector<vector<Point3f>> object_points; /* Save 3D location of checker board */
	/*内外参数*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* Intrinsic Paramerters of camera */
	vector<int> point_counts;  // number of corners in each frame
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* Distrotion parameters of camers：k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* rotation of each frame */
	vector<Mat> rvecsMat; /* translation of each frame */
	/* inialize 3D location of corners on checkerboard */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* Assume checkerboard is placed on plane z=0 (in world coordinate) */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* Initialize corner numbers of each frame, assume whole checkerboard can be seen in each frame */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}
	/* Starting Callibrate */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "Callibration is Done！\n" ;
	//Evaluate result of calibration
	cout << "\nStarting evaluate result of callibration………………\n" ;
	double total_err = 0.0; /* Sum of pixel difference in all frames */
	double err = 0.0; /* Average error in each frame */
	vector<Point2f> image_points2; /* Store measured projected location */
	cout << "Average callibration error in each frame：\n";
	fout << "Callibration error in each frame：\n";
	for (i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* According to intrinsic parameters,
		   Calculate projection according to 3D location */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* Calculate error between new and old projection location */
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "Average error in the " << i + 1 << "-th image：" << err << " pixel" << endl;
		fout << "Average error in the " << i + 1 << "-th image：" << err << " pixel" << endl;
	}
	std::cout << "Sum of average error：" << total_err / image_count << " pixel" << endl;
	fout << "Sum of average error：" << total_err / image_count << " pixel" << endl;
	std::cout << "Evaluation is done！" << endl;
	//保存定标结果  	
	std::cout << "\nStarting store the results………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* Store rotation matrix for each frame */
	fout << "Intrinsic matrix for the camers：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "Distortion parameters：\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "Rotation vector of the " << i + 1 << "-th image：" << endl;
		fout << tvecsMat[i] << endl;
		/* change roration vector into rotation matrix */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "Rotation matrix of the " << i + 1 << "-th image：" << endl;
		fout << rotation_matrix << endl;
		fout << "Rotation matrix of the " << i + 1 << "-th image：" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	std::cout << "Saving is Done" << endl;
	fout << endl;
	system("pause");
	return 0;
}
