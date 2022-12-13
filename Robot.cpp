#include "stdafx.h"						// opencv libs

#include "Robot.h"
#include <cmath>						// for trig funcs

#include <opencv2/aruco/charuco.hpp>	//

#include "cvui.h"

//#define CAM_ON
#define STEPS	75


/**
 * @brief Construct a new CRobot::CRobot object
 * 
 */
CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1280, 720);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	//////////////////////
	// init scara positions
	q1 = 0;
	q2 = 0;
	q3 = 0;
	_tip_height = 0;
	_follow_end = false;
	_track_marker = false;
	_trajOn = false;
	traj_index = 0;
	mtrack_index = 0;
	_follow_marker = false;

	j0_traj = jTraj(0, -180, 0, 0, STEPS);
	j1_traj = jTraj(0, 90, 0, 0, STEPS);
	j2_traj = jTraj(0, 90, 0, 0, STEPS);
	j3_traj = jTraj(0, 150, 0, 0, STEPS);
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

		line(im, pt1, pt2, colour, 2);
	}
}

bool CRobot::drawBox2(Mat& im, std::vector <vector<Mat>> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;
	std::vector <Point3f> box3dVec;
	std::vector <Vec3d> r_tvect;

	cv::Scalar colours[4] = { CV_RGB(255, 255, 255),CV_RGB(255, 0, 0) ,CV_RGB(0, 255, 0) ,CV_RGB(0, 0, 255) };

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	if (_virtualcam.transform_charuco(_canvas, r_tvect))
	{
		Mat extrin_re = _virtualcam.get_extrinsic();
		Mat intrin_re = _virtualcam.get_intrinsic();
		//std::cout << intrin_re << "\n\n" << extrin_re << "\n\n";
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

				line(im, pt1, pt2, colours[boxindex % 4], 2);
			}

			box3dVec.clear(); // clear 3D point vector for next box
		}
		return true;
	}
	return false;
}

void CRobot::drawCoord_cam(Mat& im, std::vector<Mat> coord3d)
{
	std::vector<Point2f> box2d;
	Mat extrin_re = _virtualcam.get_extrinsic();
	Mat intrin_re = _virtualcam.get_intrinsic();

	std::vector <Point3f> box3dVec;

	// convert to vec3d
	for (int pointindex = 0; pointindex < coord3d.size(); pointindex++)
		box3dVec.push_back(Point3f{ coord3d.at(pointindex).at<float>(0),coord3d.at(pointindex).at<float>(1), coord3d.at(pointindex).at<float>(2) });

	// transform 3D points to 2D on camera plane
	projectPoints(box3dVec, _virtualcam._rvec, _virtualcam._tvec, intrin_re, extrin_re, box2d);

	// connect points with lines
	line(im, box2d.at(0), box2d.at(1), CV_RGB(255, 0, 0), 2); // X=RED
	line(im, box2d.at(0), box2d.at(2), CV_RGB(0, 255, 0), 2); // Y=GREEN
	line(im, box2d.at(0), box2d.at(3), CV_RGB(0, 0, 255), 2); // Z=BLUE
}

void CRobot::drawCoord_cam2(Mat& im, std::vector<Mat> coord3d)
	{
	Point2f O, X, Y, Z;

	// transform 3D points to 2D on camera plane
	_virtualcam.transform_to_image_cam(coord3d.at(0), O);
	_virtualcam.transform_to_image_cam(coord3d.at(1), X);
	_virtualcam.transform_to_image_cam(coord3d.at(2), Y);
	_virtualcam.transform_to_image_cam(coord3d.at(3), Z);

	// connect points with lines
	line(im, O, X, CV_RGB(255, 0, 0), 2); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 2); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 2); // Z=BLUE
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
	line(im, O, X, CV_RGB(255, 0, 0), 2); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 2); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 2); // Z=BLUE

	}



//void CRobot::start_vidcap()
//{
//	_inputVideo.open(0);
//	_inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//	_inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
//}

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

// LAB 4 /////////////////////////////////////////////////////////////
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
	float deg = 3;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

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

	cv::imshow(CANVAS_NAME, _canvas);
}

/// LAB 5 /////////////////////////////////////
void CRobot::create_scara_robot()
{
	_simple_robot.clear();

	// create 4 boxes for robot
	std::vector <Mat> base = createBox(0.05, 0.15, 0.05);
	std::vector <Mat> first = createBox(0.15, 0.05, 0.05);
	std::vector <Mat> second = createBox(0.15, 0.05, 0.05);
	std::vector <Mat> third = createBox(0.15, 0.05, 0.05);

	// transform boxes to their positions
	transformPoints(base, createHT(Vec3d(0, 0.075, 0), Vec3d(0, 0, 0)));
	transformPoints(first, createHT(Vec3d(0.075, 0, 0), Vec3d(0, 0, 0)));
	transformPoints(second, createHT(Vec3d(0.075, 0, 0), Vec3d(0, 0, 0)));
	transformPoints(third, createHT(Vec3d(-0.075, 0, 0), Vec3d(0, 0, 0)));

	// push onto Mat vector member
	_simple_robot.push_back(base);
	_simple_robot.push_back(first);
	_simple_robot.push_back(second);
	_simple_robot.push_back(third);
}

void CRobot::draw_scara_robot()
{
	// Create BG mat
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//if (is_param_changed())
	//	{
		create_scara_robot();

		// create coordinates
		std::vector<Mat> O = createCoord();
		std::vector<Mat> OA = createCoord();
		std::vector<Mat> OB = createCoord();
		std::vector<Mat> OC = createCoord();
		std::vector<Mat> OD = createCoord();
		std::vector<Mat> EFF = createCoord();

		Mat WTA, ATB, BTC, CTD, WTB, WTC, WTD, W;

		// change axis if camera ON
		(_virtualcam._camera_on) ? W = createHT(Vec3d(0, 0, 0), Vec3d(90, 0, 0)) : W = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

		// Joint control ON
		if (_follow_end) {
			// change axis location inputs for ikine if camera is ON
			if (!_virtualcam._camera_on)
				ikine(float(_eff_x)/1000, float(-_eff_z)/1000, q1, q2, q3);
			else
				ikine(float(_eff_x) / 1000, float(_eff_y) / 1000, q1, q2, q3);
			}

		WTA = createHT(Vec3d(0, 0.175, 0), Vec3d(0, q1, 0));
		ATB = createHT(Vec3d(0.15, 0, 0), Vec3d(0, q2, 0));
		BTC = createHT(Vec3d(0.15, 0, 0), Vec3d(0, 0, -90));
		CTD = createHT(Vec3d((float)_tip_height / 1000 + 0.025, 0.025, 0), Vec3d(q3, 0, 0));

		WTA = W * WTA;
		WTB = WTA * ATB;
		WTC = WTA * ATB * BTC;
		WTD = WTA * ATB * BTC * CTD;

		transformPoints(O, W);
		transformPoints(OA, WTA);
		transformPoints(OB, WTB);
		transformPoints(OC, WTC);
		transformPoints(OD, WTD);

		// Transform Robot Parts
		transformPoints(_simple_robot.at(0), W);
		transformPoints(_simple_robot.at(1), WTA);
		transformPoints(_simple_robot.at(2), WTB);
		transformPoints(_simple_robot.at(3), WTD);
		//}

		// Camera ON
		if (_virtualcam._camera_on) {

			// draw coordinate
			if (drawBox2(_canvas, _simple_robot, CV_RGB(255, 0, 0)))
				{
				//drawCoord_cam(_canvas, O);


				/*drawCoord_cam(_canvas, OA);///////////////////////////////
				drawCoord_cam(_canvas, OB);
				drawCoord_cam(_canvas, OC);*/
				drawCoord_cam(_canvas, OD);

				}
			
			// joint control off
			if (!_follow_end)
				{
				_eff_x = int(OD.at(0).at<float>(0) * 1000);
				_eff_y = int(OD.at(0).at<float>(1) * 1000);
				_eff_z = int(OD.at(0).at<float>(2) * 1000);

				// calculate end effector pose angle
				Mat Reverse = createHT(Vec3d(float(_eff_x) / 1000, float(_eff_y) / 1000, float(_eff_z) / 1000), Vec3d(0, 0, 0));
				transformPoints(OD, Reverse.inv());
				_eff_theta = atan2(OD.at(2).at<float>(1), OD.at(2).at<float>(0)) * 180 / 3.14159;
				}
			cv::Mat R_board, rvec_inv;
			Mat tvec_inv;
			Mat R_marker;
			// Track Aruco Marker Position and move end effector using Inverse Kinematics
			if (_track_marker) {
				//if (_virtualcam._trackerIds.size() > 0)
				//	{
				//	// Invert board pose transform (rvec and tvec)
				//	//cv::Mat R_board, rvec_inv;
				//	cv::Rodrigues(_virtualcam._rvec, R_board);
				//	Mat tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
				//	Rodrigues(R_board.inv(), rvec_inv);

				//	// compose transform from board inverse and marker pose
				//	Vec3d rvec_out, tvec_out;
				//	composeRT( _virtualcam.rvecs_marker.at(0), _virtualcam.tvecs_marker.at(0), rvec_inv, tvec_inv, rvec_out, tvec_out);

				//	// get alpha rotation of marker 
				//	Mat R_marker;
				//	cv::Rodrigues(rvec_out, R_marker);
				//	double beta = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
				//	double alpha = atan2(R_marker.at<double>(3) / cos(beta), R_marker.at<double>(0) / cos(beta));
				//	//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
				//	//gamma = gamma * 180 / 3.14159; // dont need
				//	alpha = alpha * 180 / 3.14159;
				//	//beta = beta * 180 / 3.14159; // dont need

				//	// set joint angles to move end effector to marker position on board
				//	ikine(tvec_out[0], tvec_out[1], q1, q2, q3);

				//	// set end effector z-height to marker height
				//	_tip_height = 150-(tvec_out[2]*1000);

				//	// set end effector rotation to marker rotation
				//	q3 = ((q1 + q2) - alpha);

				//	}
				Vec3d rvec_out_curr, tvec_out_curr;
				Vec3d rvec_out_next, tvec_out_next;
				double beta_curr, alpha_curr, beta_next, alpha_next;
				static int STATE = 0;
				static std::vector<cv::Vec3d> rvecs_marker_s, tvecs_marker_s;
				if (_follow_marker)
					{
					if (_virtualcam.rvecs_marker.size() == 3)
						{
						switch (STATE) {

							case 0:
								// Invert board pose transform (rvec and tvec)
								
								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose
								
								composeRT(_virtualcam.rvecs_marker.at(0), _virtualcam.tvecs_marker.at(0), rvec_inv, tvec_inv, rvec_out_curr, tvec_out_curr);

								// get alpha rotation of marker 
								
								cv::Rodrigues(rvec_out_curr, R_marker);
								beta_curr = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_curr = atan2(R_marker.at<double>(3) / cos(beta_curr), R_marker.at<double>(0) / cos(beta_curr));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_curr = alpha_curr * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board
								ikine(tvec_out_curr[0], tvec_out_curr[1], j0_curr, j1_curr, j2_curr);

								// set end effector z-height to marker height
								j3_curr = 150 - (tvec_out_curr[2] * 1000);

								// set end effector rotation to marker rotation
								j2_curr = ((q1 + q2) - alpha_curr);

								////// find next marker stuff
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose
								
								composeRT(_virtualcam.rvecs_marker.at(1), _virtualcam.tvecs_marker.at(1), rvec_inv, tvec_inv, rvec_out_next, tvec_out_next);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_next, R_marker);
								beta_next = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_next = atan2(R_marker.at<double>(3) / cos(beta_next), R_marker.at<double>(0) / cos(beta_next));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_next = alpha_next * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board
			
								ikine(tvec_out_next[0], tvec_out_next[1], j0_next, j1_next, j2_next);

								// set end effector z-height to marker height
								j3_next = 150 - (tvec_out_next[2] * 1000);

								// set end effector rotation to marker rotation
								j2_next = ((q1 + q2) - alpha_next);

								j0_traj = jTraj(j0_curr, j0_next, 0, 0, STEPS);
								j1_traj = jTraj(j1_curr, j1_next, 0, 0, STEPS);
								j2_traj = jTraj(j2_curr, j2_next, 0, 0, STEPS);
								j3_traj = jTraj(j3_curr, j3_next, 0, 0, STEPS);
								//alpha_traj = jTraj(0, 150, 0, 0, STEPS);

								STATE++;
								break;
							case 1:
								if (mtrack_index < j0_traj.size() - 1)
									{
									q1 = j0_traj.at(mtrack_index);
									q2 = j1_traj.at(mtrack_index);
									q3 = j2_traj.at(mtrack_index);
									_tip_height = j3_traj.at(mtrack_index);
									mtrack_index++;
									}
								else
									STATE++;
								break;

							case 2:
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(1), _virtualcam.tvecs_marker.at(1), rvec_inv, tvec_inv, rvec_out_curr, tvec_out_curr);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_curr, R_marker);
								beta_curr = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_curr = atan2(R_marker.at<double>(3) / cos(beta_curr), R_marker.at<double>(0) / cos(beta_curr));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_curr = alpha_curr * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board
								ikine(tvec_out_curr[0], tvec_out_curr[1], j0_curr, j1_curr, j2_curr);

								// set end effector z-height to marker height
								j3_curr = 150 - (tvec_out_curr[2] * 1000);

								// set end effector rotation to marker rotation
								j2_curr = ((q1 + q2) - alpha_curr);

								////// find next marker stuff
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(2), _virtualcam.tvecs_marker.at(2), rvec_inv, tvec_inv, rvec_out_next, tvec_out_next);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_next, R_marker);
								beta_next = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_next = atan2(R_marker.at<double>(3) / cos(beta_next), R_marker.at<double>(0) / cos(beta_next));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_next = alpha_next * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board

								ikine(tvec_out_next[0], tvec_out_next[1], j0_next, j1_next, j2_next);

								// set end effector z-height to marker height
								j3_next = 150 - (tvec_out_next[2] * 1000);

								// set end effector rotation to marker rotation
								j2_next = ((q1 + q2) - alpha_next);

								j0_traj = jTraj(j0_curr, j0_next, 0, 0, STEPS);
								j1_traj = jTraj(j1_curr, j1_next, 0, 0, STEPS);
								j2_traj = jTraj(j2_curr, j2_next, 0, 0, STEPS);
								j3_traj = jTraj(j3_curr, j3_next, 0, 0, STEPS);
								//alpha_traj = jTraj(0, 150, 0, 0, STEPS);

								STATE++;
								mtrack_index = 0;
								break;
							case 3:
								
								if (mtrack_index < j0_traj.size() - 1)
									{
									q1 = j0_traj.at(mtrack_index);
									q2 = j1_traj.at(mtrack_index);
									q3 = j2_traj.at(mtrack_index);
									_tip_height = j3_traj.at(mtrack_index);
									mtrack_index++;
									}
								else
									STATE++;
							case 4:
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(2), _virtualcam.tvecs_marker.at(2), rvec_inv, tvec_inv, rvec_out_curr, tvec_out_curr);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_curr, R_marker);
								beta_curr = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_curr = atan2(R_marker.at<double>(3) / cos(beta_curr), R_marker.at<double>(0) / cos(beta_curr));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_curr = alpha_curr * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board
								ikine(tvec_out_curr[0], tvec_out_curr[1], j0_curr, j1_curr, j2_curr);

								// set end effector z-height to marker height
								j3_curr = 150 - (tvec_out_curr[2] * 1000);

								// set end effector rotation to marker rotation
								j2_curr = ((q1 + q2) - alpha_curr);

								////// find next marker stuff
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(1), _virtualcam.tvecs_marker.at(1), rvec_inv, tvec_inv, rvec_out_next, tvec_out_next);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_next, R_marker);
								beta_next = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_next = atan2(R_marker.at<double>(3) / cos(beta_next), R_marker.at<double>(0) / cos(beta_next));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_next = alpha_next * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board

								ikine(tvec_out_next[0], tvec_out_next[1], j0_next, j1_next, j2_next);

								// set end effector z-height to marker height
								j3_next = 150 - (tvec_out_next[2] * 1000);

								// set end effector rotation to marker rotation
								j2_next = ((q1 + q2) - alpha_next);

								j0_traj = jTraj(j0_curr, j0_next, 0, 0, STEPS);
								j1_traj = jTraj(j1_curr, j1_next, 0, 0, STEPS);
								j2_traj = jTraj(j2_curr, j2_next, 0, 0, STEPS);
								j3_traj = jTraj(j3_curr, j3_next, 0, 0, STEPS);
								//alpha_traj = jTraj(0, 150, 0, 0, STEPS);
								mtrack_index = 0;
								STATE++;
								break;
							case 5:
								if (mtrack_index < j0_traj.size() - 1)
									{
									q1 = j0_traj.at(mtrack_index);
									q2 = j1_traj.at(mtrack_index);
									q3 = j2_traj.at(mtrack_index);
									_tip_height = j3_traj.at(mtrack_index);
									mtrack_index++;
									}
								else
									STATE++;
								break;

							case 6:
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(1), _virtualcam.tvecs_marker.at(1), rvec_inv, tvec_inv, rvec_out_curr, tvec_out_curr);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_curr, R_marker);
								beta_curr = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_curr = atan2(R_marker.at<double>(3) / cos(beta_curr), R_marker.at<double>(0) / cos(beta_curr));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_curr = alpha_curr * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board
								ikine(tvec_out_curr[0], tvec_out_curr[1], j0_curr, j1_curr, j2_curr);

								// set end effector z-height to marker height
								j3_curr = 150 - (tvec_out_curr[2] * 1000);

								// set end effector rotation to marker rotation
								j2_curr = ((q1 + q2) - alpha_curr);

								////// find next marker stuff
								// Invert board pose transform (rvec and tvec)

								cv::Rodrigues(_virtualcam._rvec, R_board);
								tvec_inv = -R_board.inv() * Mat(_virtualcam._tvec);
								Rodrigues(R_board.inv(), rvec_inv);

								// compose transform from board inverse and marker pose

								composeRT(_virtualcam.rvecs_marker.at(0), _virtualcam.tvecs_marker.at(0), rvec_inv, tvec_inv, rvec_out_next, tvec_out_next);

								// get alpha rotation of marker 

								cv::Rodrigues(rvec_out_next, R_marker);
								beta_next = atan2(-(R_marker.at<double>(6)), sqrt(pow(R_marker.at<double>(0), 2) + pow(R_marker.at<double>(3), 2)));
								alpha_next = atan2(R_marker.at<double>(3) / cos(beta_next), R_marker.at<double>(0) / cos(beta_next));
								//double gamma = atan2(R.at<double>(7) / cos(beta), R.at<double>(8) / cos(beta)); // dont need
								//gamma = gamma * 180 / 3.14159; // dont need
								alpha_next = alpha_next * 180 / 3.14159;
								//beta = beta * 180 / 3.14159; // dont need

								// set joint angles to move end effector to marker position on board

								ikine(tvec_out_next[0], tvec_out_next[1], j0_next, j1_next, j2_next);

								// set end effector z-height to marker height
								j3_next = 150 - (tvec_out_next[2] * 1000);

								// set end effector rotation to marker rotation
								j2_next = ((q1 + q2) - alpha_next);

								j0_traj = jTraj(j0_curr, j0_next, 0, 0, STEPS);
								j1_traj = jTraj(j1_curr, j1_next, 0, 0, STEPS);
								j2_traj = jTraj(j2_curr, j2_next, 0, 0, STEPS);
								j3_traj = jTraj(j3_curr, j3_next, 0, 0, STEPS);
								//alpha_traj = jTraj(0, 150, 0, 0, STEPS);

								STATE++;
								mtrack_index = 0;
								break;
							case 7:

								if (mtrack_index < j0_traj.size() - 1)
									{
									q1 = j0_traj.at(mtrack_index);
									q2 = j1_traj.at(mtrack_index);
									q3 = j2_traj.at(mtrack_index);
									_tip_height = j3_traj.at(mtrack_index);
									mtrack_index++;
									}
								else
									STATE = 0;
							}
						}
					else
						STATE = 0;
					}
				
				}


			flip(_canvas, _canvas, 1);
			}
		else {
			// draw robot boxes
			drawBox(_canvas, _simple_robot.at(0), CV_RGB(255, 255, 255));
			drawBox(_canvas, _simple_robot.at(1), CV_RGB(0, 255, 0));
			drawBox(_canvas, _simple_robot.at(2), CV_RGB(0, 0, 255));
			drawBox(_canvas, _simple_robot.at(3), CV_RGB(255, 0, 0));

			// draw coorditate
			drawCoord(_canvas, O);
			drawCoord(_canvas, OA);
			drawCoord(_canvas, OB);
			drawCoord(_canvas, OC);
			drawCoord(_canvas, OD);

			if (!_follow_end)
				{
				_eff_x = int((OD.at(0).at<float>(0) * 1000) + 0.5);
				_eff_y = int((OD.at(0).at<float>(1) * 1000) + 0.5);
				_eff_z = int((OD.at(0).at<float>(2) * 1000) + 0.5);


				// calculate end effector pose angle
				Mat Reverse = createHT(Vec3d(float(_eff_x) / 1000, float(_eff_y) / 1000, float(_eff_z) / 1000), Vec3d(0, 0, 0));
				transformPoints(OD, Reverse.inv());
				_eff_theta = int(atan2(OD.at(2).at<float>(2), OD.at(2).at<float>(0)) * 180 / 3.14159);
				}

			}

		// get setpoint
		// 

		/*transformPoints(EFF, createHT(Vec3d(0, 0.175, 0), Vec3d(0, 0, 0)) * fkine());
		drawCoord(_canvas, EFF);*/




	// update camera
	update_endeff_window();
	_virtualcam.update_settings(_canvas, q1, q2, q3, _tip_height);
	

	// show image in window
	cv::imshow(CANVAS_NAME, _canvas);

}

Mat CRobot::fkine()
	{
	const double conv = 3.14159265358979323846 / 180;
	return ((Mat1f(4, 4) << cos(q3*conv) * (cos(q1 * conv) * cos(q2 * conv) - sin(q1 * conv) * sin(q2 * conv)) - sin(q3 * conv) * (cos(q1 * conv) * sin(q2 * conv) + cos(q2 * conv) * sin(q1 * conv)), 0, cos(q3 * conv) * (cos(q1 * conv) * sin(q2 * conv) + cos(q2 * conv) * sin(q1 * conv)) + sin(q3 * conv) * (cos(q1 * conv) * cos(q2 * conv) - sin(q1 * conv) * sin(q2 * conv)), (3 * cos(q1 * conv)) / 20 + (7 * cos(q1 * conv) * cos(q2 * conv)) / 40 - (7 * sin(q1 * conv) * sin(q2 * conv)) / 40,
									0, 1, 0, -float(_tip_height)/1000,
		-cos(q3 * conv) * (cos(q1 * conv) * sin(q2 * conv) + cos(q2 * conv) * sin(q1 * conv)) - sin(q3 * conv) * (cos(q1) * conv * cos(q2 * conv) - sin(q1 * conv) * sin(q2 * conv)), 0, cos(q3 * conv) * (cos(q1 * conv) * cos(q2 * conv) - sin(q1 * conv) * sin(q2 * conv)) - sin(q3 * conv) * (cos(q1 * conv) * sin(q2 * conv) + cos(q2 * conv) * sin(q1 * conv)), -(3 * sin(q1 * conv)) / 20 - (7 * cos(q1 * conv) * sin(q2 * conv)) / 40 - (7 * cos(q2 * conv) * sin(q1 * conv)) / 40,
		0, 0, 0, 1
		));
	}

bool CRobot::is_param_changed()
	{
	static int th_old = -1, q1_old = -1, q2_old = -1, q3_old = -1;

	if ((_tip_height != th_old) || (q1 != q1_old) || (q2 != q2_old) || (q3 != q3_old)) {
		th_old = _tip_height;
		q1_old = q1;
		q2_old = q2;
		q3_old = q3;
		return true;
		}
	else 
		return false;
	}

void CRobot::update_jtraj_window()
	{
	// Pose window 
	Point _eff_window;
	_eff_window.x = _canvas.cols - 200;
	_eff_window.y = 300;

	cvui::window(_canvas, _eff_window.x, _eff_window.y, 200, 270, "End Effector");

	_eff_window.x += 5;;
	_eff_window.y += 20;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_x, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "X");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_y, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Y");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_z, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Z");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_theta, -180, 180);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Rot");

	_eff_window.y += 50;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Joint Control", &_follow_end);
	_eff_window.y += 25;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Track Object", &_track_marker);
	_eff_window.y += 25;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Trajectory", &_trajOn);

	}


void CRobot::update_endeff_window()
	{
	// Pose window 
	Point _eff_window;
	_eff_window.x = _canvas.cols - 200;
	_eff_window.y = 300;
	static int STATE = 0;

	cvui::window(_canvas, _eff_window.x, _eff_window.y, 200, 300, "End Effector");

	_eff_window.x += 5;;
	_eff_window.y += 20;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_x, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "X");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_y, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Y");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_z, -400, 400);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Z");

	_eff_window.y += 45;
	cvui::trackbar(_canvas, _eff_window.x, _eff_window.y, 180, &_eff_theta, -180, 180);
	cvui::text(_canvas, _eff_window.x + 180, _eff_window.y + 20, "Rot");

	_eff_window.y += 50;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Joint Control", &_follow_end);
	_eff_window.y += 25;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Track Object", &_track_marker);
	_eff_window.y += 25;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Trajectory", &_trajOn);
	_eff_window.y += 25;
	cvui::checkbox(_canvas, _eff_window.x, _eff_window.y, "Marker Follow", &_follow_marker);

	if (_trajOn)
		{
		switch (STATE)
			{
			case 0:
				if (traj_index < j0_traj.size() - 1)
					{
					q1 = j0_traj.at(traj_index);
					q2 = j1_traj.at(traj_index);
					q3 = j2_traj.at(traj_index);
					_tip_height = j3_traj.at(traj_index);
					traj_index++;
					}
				else
					STATE++;
				break;
			case 1:
				if (traj_index != 0)
					{
					q1 = j0_traj.at(traj_index);
					q2 = j1_traj.at(traj_index);
					q3 = j2_traj.at(traj_index);
					_tip_height = j3_traj.at(traj_index);
					traj_index--;
					}
				else
					STATE = 0;
				break;


			}

		}

	}

// Calculate Inverse Kinematic joint angles from x,y plane location
void CRobot::ikine(float x, float y, int &q1, int &q2, int &q3)
	{
	// calculate joint angles from MATLAB solution
	float q1f = 2 * atan2((480 * y - sqrt(-2560000 * pow(x, 4) - 5120000 * pow(x, 2) * pow(y, 2) + 272000 * pow(x, 2) - 2560000 * pow(y, 4) + 272000 * pow(y, 2) - 169)),(1600 * pow(x, 2) + 480 * x + 1600 * pow(y, 2) - 13)) * 180 / 3.14159;
	float q2f = 2 * atan2(sqrt(-(1600 * pow(x, 2) + 1600 * pow(y, 2) - 1) * (1600 * pow(x, 2) + 1600 * pow(y, 2) - 169)), (1600 * pow(x, 2) + 1600 * pow(y, 2) - 1)) *180 / 3.14159;
	
	// cast to int
	q2 = int(q2f);
	q1 = int(q1f);
	//q3 = ((q1 + q2) - _eff_theta);

	// set radius of reach boundary behaviour
	if (q1 < -1000)
		{
		q2 = 0;
		q1 = atan2(y, x) * 180 / 3.14159;
		}
	}

void CRobot::drawTraj()
	{
	// Create BG mat
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//if (is_param_changed())
	//	{
	create_scara_robot();

	// create coordinates
	std::vector<Mat> O = createCoord();
	std::vector<Mat> OA = createCoord();
	std::vector<Mat> OB = createCoord();
	std::vector<Mat> OC = createCoord();
	std::vector<Mat> OD = createCoord();
	std::vector<Mat> EFF = createCoord();

	Mat WTA, ATB, BTC, CTD, WTB, WTC, WTD, W;



	int stepnum = 10;

	vector <double> x = jTraj(0, 1, 0, 0, stepnum);
	vector <double> y = jTraj(0, 1, 0, 0, stepnum);
	vector <double> z = jTraj(0, 1, 0, 0, stepnum);
	vector <double> theta = jTraj(0, 90, 0, 0, stepnum);

	vector <double> traj_angles;

	for (int i = 0; i < x.size(); i++)
		{
		//ikine()
		}



	for (int i = 0; i < x.size(); i++)
		std::cout << x.at(i) << "\n";
	}

vector <double> CRobot::jTraj(double j0, double j1, double jd0, double jd1, int steps)
	{


		// populate polynomial interpolation matrix
		Mat M = (Mat1d(6, 6) << 0, 0, 0, 0, 0, 1,
		pow(steps, 5), pow(steps, 4), pow(steps, 3), pow(steps, 2), steps, 1,
		0, 0, 0, 0, 1, 0,
		5 * pow(steps, 4), 4 * pow(steps, 3), 3 * pow(steps, 2), 2 * steps, 1, 0,
		0, 0, 0, 2, 0, 0,
		20 * pow(steps, 3), 12 * pow(steps, 2), 6 * steps, 2, 0, 0);

		// create s(t) matrix
		Mat S = (Mat1d(6, 1) << j0, j1, jd0, jd1, 0, 0);

		// Find solution to polynomial coefficients
		Mat coeffs = M.inv() * S;

		//std::cout << coeffs << "\n";

		int time = 0;

		vector <double> result;
		int sum = 0;
		
		// generate jtraj vector
		while (time != (steps - 1))
			{
			Mat T = (Mat1d(1, 6) << pow(time, 5), pow(time, 4), pow(time, 3), pow(time, 2), time, 1);

			Mat sum =  T * coeffs;

			//std::cout << sum << "\n";

			result.push_back(sum.at<double>(0));
			time++;

			}

		return result;
	}