#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;

	cv::VideoCapture _inputVideo;


	vector<vector<Mat>> _simple_robot;

	Vec3d _trans;
	Vec3d _rot;

	//CuArm uarm;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawBox2(Mat& im, std::vector <vector <Mat>> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);


public:
	Mat createHT(Vec3d t, Vec3d r);
	CCamera _virtualcam;
	/////////////////////////////
	// Lab 3

	void create_simple_robot();
	void draw_simple_robot();

	

	/////////////////////////////
	// Lab 4
	void create_aruco_robot();
	void draw_aruco_robot();
	void start_vidcap();

	/////////////////////////////
  // Lab 5

};

