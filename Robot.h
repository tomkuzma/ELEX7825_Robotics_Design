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
	/**
	 * @brief 
	 * 
	 */
	Mat _canvas;

	

	vector<vector<Mat>> _simple_robot;

	Vec3d _trans;
	Vec3d _rot;

	//CuArm uarm;

	// forward kinematics vars
	int _tip_height; // in mm
	int q1, q2, q3;	// in deg
	int j0_curr, j1_curr, j2_curr, j3_curr;
	int j0_next, j1_next, j2_next, j3_next;


	int _eff_x, _eff_y, _eff_z, _eff_theta; // in mm

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	bool drawBox2(Mat& im, std::vector <vector <Mat>> box3d, Scalar colour);
	void drawCoord_cam(Mat& im, std::vector<Mat> coord3d);
	void drawCoord_cam2(Mat& im, std::vector<Mat> coord3d);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	vector <double> j0_traj;
	vector <double> j1_traj;
	vector <double> j2_traj;
	vector <double> j3_traj;

	vector <double> x_traj;
	vector <double> y_traj;
	vector <double> z_traj;
	vector <double> alphja_traj;

	int traj_index, mtrack_index;

	bool _follow_end, _track_marker, _trajOn, _follow_marker;

public:
	Mat createHT(Vec3d t, Vec3d r);
	CCamera _virtualcam;
	Vec3d rvec_out, tvec_out;
	/////////////////////////////
	// Lab 3

	void create_simple_robot();
	void draw_simple_robot();


	/////////////////////////////
	// Lab 4
	void create_aruco_robot();
	void draw_aruco_robot();

	/////////////////////////////
  // Lab 5
	void create_scara_robot();
	void draw_scara_robot();
	Mat fkine();
	bool is_param_changed();
	void update_endeff_window();

	//////////////////////////////
	// Lab 6
	void ikine(float x, float y, int& q1, int& q2, int& q3);

	//////////////////////////////
	// Lab 7
	void drawTraj();
	vector<double> jTraj(double j0, double j1, double jd0, double jd1, int steps);
	void update_jtraj_window();

};	

