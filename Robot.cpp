#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1280, 720);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

  ///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();
}

CRobot::~CRobot()
{
}

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	// constants
	const double conv = 3.14159265358979323846/180;
	const double alpha = r[2] * conv;
	const double beta = r[1] * conv;
	const double gamma = r[0] * conv;
	const double sa = sin(alpha);
	const double sb = sin(beta);
	const double sg = sin(gamma);
	const double ca = cos(alpha);
	const double cb = cos(beta);
	const double cg = cos(gamma);

	// matrix value calculations
	double a = ca * cb;
	double b = (ca * sb * sg) - (sa * cg);
	double c = (ca * sb * cg) + (sa * sg);
	double d = sa * cb;
	double e = (sa * sb * sg) + (ca * cg);
	double f = (sa * sb * cg) - (ca * sg);
	double g = -(sb);
	double h = cb * sg;
	double i = cb * cg;

	return ((Mat1f(4, 4) << a, b, c, t[0], d, e, f, t[1], g, h, i, t[2], 0, 0, 0, 1));
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;

	float axis_length = 0.05;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}


void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image(box3d, box2d); 

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawBox2(Mat& im, std::vector <vector<Mat>> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;
	std::vector <Point3f> box3dVec;
	std::vector <Vec3d> r_tvect;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	if (_virtualcam.transform_charuco(_inputVideo, _canvas, r_tvect))
	{
		Mat extrin_re = _virtualcam.get_extrinsic();
		Mat intrin_re = _virtualcam.get_intrinsic();

		for (int boxindex = 0; boxindex < box3d.size(); boxindex++)
		{
			// convert box3d Mat to Point3f
			for (int pointindex = 0; pointindex < box3d.at(boxindex).size(); pointindex++)
				box3dVec.push_back(Point3f{ box3d.at(boxindex).at(pointindex).at<float>(0),box3d.at(boxindex).at(pointindex).at<float>(1), box3d.at(boxindex).at(pointindex).at<float>(2)});
					
			// get 2d points
			projectPoints(box3dVec, r_tvect.at(0), r_tvect.at(1), intrin_re, extrin_re, box2d);

			// draw lines between 2D box points
			for (int i = 0; i < 12; i++)
			{
				Point pt1 = box2d.at(draw_box1[i]);
				Point pt2 = box2d.at(draw_box2[i]);

				line(im, pt1, pt2, colour, 2);
			}

			box3dVec.clear(); // clear 3D point vector for next box
		}

	}

}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	// transform 3D points to 2D on camera plane
	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	// connect points with lines
	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
	
}

void CRobot::start_vidcap()
{
	_inputVideo.open(0);
	_inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	_inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
}

void CRobot::create_simple_robot()
{
	// create 5 boxes for robot
	std::vector <Mat> bottom = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> middle = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> left = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> right = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> head = createBox(0.05, 0.05, 0.05);

	// transform boxes to their positions
	transformPoints(middle, createHT(Vec3d(0, 0.05, 0), Vec3d(0, 0, 0)));
	transformPoints(left, createHT(Vec3d(-0.05, 0.1, 0), Vec3d(0, 0, 0)));
	transformPoints(right, createHT(Vec3d(0.05, 0.1, 0), Vec3d(0, 0, 0)));
	transformPoints(head, createHT(Vec3d(0, 0.15, 0), Vec3d(0, 0, 0)));

	// push onto Mat vector member
	_simple_robot.push_back(bottom);
	_simple_robot.push_back(middle);
	_simple_robot.push_back(left);
	_simple_robot.push_back(right);
	_simple_robot.push_back(head);
	
}

void CRobot::draw_simple_robot()
{
	// Create BG mat
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// create world coordinates
	std::vector<Mat> O = createCoord();

	// draw coorditate
	drawCoord(_canvas, O);

	// draw robot boxes
	drawBox(_canvas, _simple_robot.at(0), CV_RGB(255, 0, 0));
	drawBox(_canvas, _simple_robot.at(1), CV_RGB(0, 255, 0));
	drawBox(_canvas, _simple_robot.at(2), CV_RGB(0, 0, 255));
	drawBox(_canvas, _simple_robot.at(3), CV_RGB(0, 0, 255));
	drawBox(_canvas, _simple_robot.at(4), CV_RGB(255, 0, 255));

	// update camera
	_virtualcam.update_settings(_canvas);

	// show image in window
	cv::imshow(CANVAS_NAME, _canvas);

}

void CRobot::create_aruco_robot()
{
	// create 5 boxes for robot
	std::vector <Mat> bottom = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> middle = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> left = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> right = createBox(0.05, 0.05, 0.05);
	std::vector <Mat> head = createBox(0.05, 0.05, 0.05);

	// transform boxes to their positions
	transformPoints(middle, createHT(Vec3d(0, 0, 0.05), Vec3d(0, 0, 0)));
	transformPoints(left, createHT(Vec3d(-0.05, 0, 0.1), Vec3d(0, 0, 0)));
	transformPoints(right, createHT(Vec3d(0.05, 0, 0.1), Vec3d(0, 0, 0)));
	transformPoints(head, createHT(Vec3d(0, 0, 0.15), Vec3d(0, 0, 0)));

	// push onto Mat vector member
	_simple_robot.push_back(bottom);
	_simple_robot.push_back(middle);
	_simple_robot.push_back(left);
	_simple_robot.push_back(right);
	_simple_robot.push_back(head);
}

void CRobot::draw_aruco_robot()
{
	float deg = 1;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	Mat flipped;

	// rotate boxes
	transformPoints(_simple_robot.at(0), createHT(Vec3d(0, 0, 0), Vec3d(0, 0, deg)));
	transformPoints(_simple_robot.at(1), createHT(Vec3d(0, 0, 0), Vec3d(0, 0, deg)));
	transformPoints(_simple_robot.at(2), createHT(Vec3d(0, 0, 0), Vec3d(0, 0, deg)));
	transformPoints(_simple_robot.at(3), createHT(Vec3d(0, 0, 0), Vec3d(0, 0, deg)));
	transformPoints(_simple_robot.at(4), createHT(Vec3d(0, 0, 0), Vec3d(0, 0, deg)));

	// draw boxes on board pose
	drawBox2(_canvas, _simple_robot, CV_RGB(255, 0, 0));

	// flip image horizontally
	flip(_canvas, _canvas, 1);

	// update trackbars
	_virtualcam.update_settings(_canvas);

	// increment rotation
	deg += 1;

	cv::imshow(CANVAS_NAME, _canvas);
}
