#include "stdafx.h"

#include "Camera.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1280, 720));

}

CCamera::~CCamera()
{
}

void CCamera::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 0;

	_cam_setting_x = 0; // units in mm
	_cam_setting_y = 0; // units in mm
	_cam_setting_z = 500; // units in mm

	_cam_setting_roll = 0; // units in degrees
	_cam_setting_pitch = 180; // units in degrees
	_cam_setting_yaw = 180; // units in degrees

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);
	/*load_camparam("cam_param.xml", _cam_real_intrinsic, _cam_real_dist_coeff);*/
	
	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();

	load_camparam("cam_param.xml", _cam_real_intrinsic, _cam_real_dist_coeff);

	_animate_flg = false;
	_camera_on = false;
}

void CCamera::calculate_intrinsic()
{
	// matrix const
	const float a = (_cam_setting_f / (1000 * _pixel_size ));

	// populate intrinsic matrix
	_cam_virtual_intrinsic = ((Mat1f(3, 4) << a, 0, _principal_point.x, 0, 0, a, _principal_point.y, 0, 0, 0, 1, 0));
}

void CCamera::calculate_extrinsic()
{
	// constants
	const double conv = 3.14159265358979323846 / 180;
	const double alpha = _cam_setting_yaw * conv;
	const double beta = _cam_setting_pitch * conv;
	const double gamma = _cam_setting_roll * conv;
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

	// populate extrinsic matrix
	_cam_virtual_extrinsic = (Mat1f(4, 4) << a, b, c, float(_cam_setting_x) / 1000, d, e, f, float(_cam_setting_y) / 1000, g, h, i, float(_cam_setting_z) / 1000, 0, 0, 0, 1);
}

bool CCamera::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

bool CCamera::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);
	
	if (!fs.isOpened())
	{
		return false;
	}
	
	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

void CCamera::createChArUcoBoard()
{
	Mat im;
	float size_square = 0.04; // user specified
	float size_mark = 0.02; // user specified
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_id);
	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
	
	board->draw(cv::Size(600, 500), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCamera::calibrate_board(int cam_id)
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;
  
	float size_aruco_square = 34.63/1000; // MEASURE THESE
	float size_aruco_mark = 20.7/1000; // MEASURE THESE

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	VideoCapture inputVideo;
	inputVideo.open(cam_id);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	// Collect data from live video 
	while (inputVideo.grab()) 
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(draw_im);
		
		// First pass detect markers
		aruco::detectMarkers(im, dictionary, corners, corner_ids, detectorParams, rejected_corners);
		// Second pass detect markers
		aruco::refineDetectedMarkers(im, board, corners, corner_ids, rejected_corners);

		// Refine charuco corners
		if (corner_ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0)
		{
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0)
		{
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}
		
		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0) 
		{
			cout << "Frame captured" << endl;
			calib_corner.push_back(corners);
			calib_id.push_back(corner_ids);
			calib_im.push_back(im);
			calib_im_size = im.size();
		}
	}

	if (calib_id.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(calib_corner.size());
	for (unsigned int i = 0; i < calib_corner.size(); i++) {
		markerCounterPerFrame.push_back((int)calib_corner[i].size());
		for (unsigned int j = 0; j < calib_corner[i].size(); j++) {
			allCornersConcatenated.push_back(calib_corner[i][j]);
			allIdsConcatenated.push_back(calib_id[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, calib_im_size, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

	// prepare data for charuco calibration
	int nFrames = (int)calib_corner.size();
	vector< Mat > allCharucoCorners;
	vector< Mat > allCharucoIds;
	vector< Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for (int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		Mat currentCharucoCorners, currentCharucoIds;
		aruco::interpolateCornersCharuco(calib_corner[i], calib_id[i], calib_im[i], charucoboard,
			currentCharucoCorners, currentCharucoIds, cameraMatrix,
			distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(calib_im[i]);
	}

	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	// calibrate camera using charuco
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, calib_im_size, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = save_camparam("cam_param.xml", cameraMatrix, distCoeffs);
	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Rep Error Aruco: " << arucoRepErr << endl;
	cout << "Calibration saved to " << "cam_param.xml" << endl;

	// show interpolated charuco corners for debugging
		for (unsigned int frame = 0; frame < filteredImages.size(); frame++) 
		{
			Mat imageCopy = filteredImages[frame].clone();
			
			if (calib_id[frame].size() > 0) {

				if (allCharucoCorners[frame].total() > 0) 
				{
					aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],allCharucoIds[frame]);
				}
			}

			imshow("out", imageCopy);
			char key = (char)waitKey(0);
			if (key == 27) break;
	}
}

void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	// calculate homogeneous point from camera transform
	Mat C = _cam_virtual_intrinsic * _cam_virtual_extrinsic * pt3d_mat;
	Point3d P = C.clone(); // cast to Point
	
	// projection calc for 2D point
	pt.x = int(P.x / P.z);
	pt.y = int(P.y / P.z);

}

void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	// calculate camera transformation matrix
	Mat C = _cam_virtual_intrinsic * _cam_virtual_extrinsic;
	Mat P; // for pushing points onto vector

	for (int i = 0; i < pts3d_mat.size(); i++) {
		P = C * pts3d_mat.at(i);
		Point3d Pts = P.clone();

		pts2d.push_back(Point2f(int(Pts.x / Pts.z), int(Pts.y / Pts.z)));

	}
}

void CCamera::update_settings(Mat &im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");
	
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board);

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	cvui::update();

	//////////////////////////////
	// Update camera model
	/*_cam_setting_pitch++;
	_cam_setting_roll++;
	_cam_setting_yaw++;*/
	calculate_intrinsic();
	calculate_extrinsic();
}

void CCamera::update_settings(Mat& im, int &j0, int &j1, int &j2, int &j3)
	{
	static int STATE = 0; 
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board);

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
		{
		init(im.size());
		}

	// Pose window 
	Point _pose_window;
	_pose_window.x = im.cols - 200;
	_pose_window.y = 0;

	cvui::window(im, _pose_window.x, _pose_window.y, 200, 300, "Pose Settings");

	_pose_window.x += 5;;
	_pose_window.y += 20;
	cvui::trackbar(im, _pose_window.x, _pose_window.y, 180, &j0, -180, 180);
	cvui::text(im, _pose_window.x + 180, _pose_window.y + 20, "j0");

	_pose_window.y += 45;
	cvui::trackbar(im, _pose_window.x, _pose_window.y, 180, &j1, -180, 180);
	cvui::text(im, _pose_window.x + 180, _pose_window.y + 20, "j1");

	_pose_window.y += 45;
	cvui::trackbar(im, _pose_window.x, _pose_window.y, 180, &j2, -180, 180);
	cvui::text(im, _pose_window.x + 180, _pose_window.y + 20, "j2");

	_pose_window.y += 45;
	cvui::trackbar(im, _pose_window.x, _pose_window.y, 180, &j3, 0, 150);
	cvui::text(im, _pose_window.x + 180, _pose_window.y + 20, "j3");

	_pose_window.y += 50;
	// reset pose
	if (cvui::button(im, _pose_window.x, _pose_window.y, 100, 30, "Reset"))
		{
		j0 = 0;
		j1 = 0;
		j2 = 0;
		j3 = 0;
		}

	// animate joints
	if (cvui::button(im, _pose_window.x + 100, _pose_window.y, 100, 30, "Animate"))
		{
		_animate_flg = true;
		j0 = 0;
		j1 = 0;
		j2 = 0;
		j3 = 0;
		STATE = 0;
		}

	//_pose_window.x -= 50;
	_pose_window.y += 45;
	cvui::checkbox(im, _pose_window.x, _pose_window.y, "Camera On", &_camera_on);

	// do animation with FSM
	if (_animate_flg) {
		switch (STATE)
			{
			case 0: (j0 != 180) ? j0+=5 : STATE++;
				break;
			case 1: (j0 != -180) ? j0-=5 : STATE++;
				break;
			case 2: (j0 != 0) ? j0+=5 : STATE++;
				break;
			case 3: (j1 != 180) ? j1+=5 : STATE++;
				break;
			case 4: (j1 != -180) ? j1-=5 : STATE++;
				break;
			case 5: (j1 != 0) ? j1+=5 : STATE++;
				break;
			case 6: (j2 != 360) ? j2+=5 : STATE++;
				break;
			case 7: (j3 != 150) ? j3+=5 : STATE++;
				break;
			case 8: (j3 != 0) ? j3-=5 : STATE++;
				break;
			default: _animate_flg = false;
				break;
			}


		}

	cvui::update();

	//////////////////////////////
	// Update camera model
	/*_cam_setting_pitch++;
	_cam_setting_roll++;
	_cam_setting_yaw++;*/
	calculate_intrinsic();
	calculate_extrinsic();
	}

void CCamera::detectCharucoBoardWithCalibrationPose()
{
	cv::VideoCapture inputVideo;
	inputVideo.open(0);
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	//cv::Mat _cam_real_intrinsic, _cam_real_dist_coeff;
	std::string filename = "calib.txt";
	bool readOk = load_camparam("cam_param.xml", _cam_real_intrinsic, _cam_real_dist_coeff);
	if (!readOk) {
		std::cerr << "Invalid camera file" << std::endl;
	}
	else {
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
		cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
		while (inputVideo.grab()) {
			cv::Mat image;
			cv::Mat imageCopy;
			inputVideo.retrieve(image);
			image.copyTo(imageCopy);
			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f> > markerCorners;
			cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
			// if at least one marker detected
			if (markerIds.size() > 0) {
				cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
				std::vector<cv::Point2f> charucoCorners;
				std::vector<int> charucoIds;
				cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, _cam_real_intrinsic, _cam_real_dist_coeff);
				// if at least one charuco corner detected
				if (charucoIds.size() > 0) {
					cv::Scalar color = cv::Scalar(255, 0, 0);
					cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
					cv::Vec3d rvec, tvec;
					bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec);
					// if charuco pose is valid
					if (valid)
						cv::drawFrameAxes(imageCopy, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec, 0.1f);
				}
			}
			cv::imshow("out", imageCopy);
			char key = (char)cv::waitKey(30);
			if (key == 27)
				//cv::destroyAllWindows();
				break;
			
		}
	}
}

bool CCamera::transform_charuco(VideoCapture vid, Mat &im, vector <Vec3d> &rtvec)
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
	cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
	vid.grab();
	cv::Mat image;
	cv::Mat imageCopy;
	vid.retrieve(image);
	//cv::flip(image, image, 1);
	image.copyTo(im);
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f> > markerCorners;
	cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
	// if at least one marker detected
	if (markerIds.size() > 0) {
		cv::aruco::drawDetectedMarkers(im, markerCorners, markerIds);
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> charucoIds;
		cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, _cam_real_intrinsic, _cam_real_dist_coeff);
		// if at least one charuco corner detected
		if (charucoIds.size() > 0) {
			cv::Scalar color = cv::Scalar(255, 0, 0);
			cv::aruco::drawDetectedCornersCharuco(im, charucoCorners, charucoIds, color);
			cv::Vec3d rvec, tvec;
			bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec);
			// if charuco pose is valid
			if (valid) {
			cv::drawFrameAxes(im, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec, 0.1f);

			// set board pose to sliders on GUI
			_cam_setting_x = int(tvec[0] * 1000); // units in mm
			_cam_setting_y = int(tvec[1] * 1000); // units in mm
			_cam_setting_z = int(tvec[2] * 1000); // units in mm

			rtvec.push_back(rvec);
			rtvec.push_back(tvec);

			_rvec = rvec;
			_tvec = tvec;

			return true;
			}
		}
	}
	return false;


}

Mat CCamera::get_intrinsic()
{
	return _cam_real_intrinsic;
}

Mat CCamera::get_extrinsic()
{
	return _cam_real_extrinsic;
}

