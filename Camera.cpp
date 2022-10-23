#include "stdafx.h"

#include "Camera.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

#define ESC 27

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1000, 600));

	_rvec = {0,0,0};
	_tvec = {0,0,0};
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

	_cam_setting_roll = -163; // units in degrees
	_cam_setting_pitch = 0; // units in degrees
	_cam_setting_yaw = 0; // units in degrees

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);
	
	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();

	//////////////////////////////////////
	// Real Camera Extrinsic

	load_camparam("cam_param.xml", _cam_real_intrinsic, _cam_real_dist_coeff);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lab 3
////////////////////////////////////////////////////////////////////////////////////////////////////////

//calculate intrinsic image transform matrix - added by Seamus Finlayson
void CCamera::calculate_intrinsic()
{
	Mat pixel_principal_mat = (Mat1f(3, 3) << 
		1/_pixel_size, 0, (float)_principal_point.x,
		0, 1/_pixel_size, (float)_principal_point.y,
		0, 0, 1
		);

	Mat focal_mat = (Mat1f(3, 4) << 
		(float)_cam_setting_f / 1000, 0, 0, 0,
		0, (float)_cam_setting_f / 1000, 0, 0,
		0, 0, 1, 0
		);

	_cam_virtual_intrinsic = pixel_principal_mat * focal_mat;
	
}

//calculate intrinsic image transform matrix - added by Seamus Finlayson
void CCamera::calculate_extrinsic()
{
	float roll = (float)_cam_setting_roll / 57.2957795131; //convert from degrees to radians
	float pitch = (float)_cam_setting_pitch / 57.2957795131; //convert from degrees to radians
	float yaw = (float)_cam_setting_yaw / 57.2957795131; //convert from degrees to radians

	double r11 = cos(yaw) * cos(pitch);
	double r12 = (cos(yaw) * sin(pitch) * sin(roll)) - (sin(yaw) * cos(roll));
	double r13 = (cos(yaw) * sin(pitch) * cos(roll)) + (sin(yaw) * sin(roll));

	double r21 = sin(yaw) * cos(pitch);
	double r22 = (sin(yaw) * sin(pitch) * sin(roll)) + (cos(yaw) * cos(roll));
	double r23 = (sin(yaw) * sin(pitch) * cos(roll)) - (cos(yaw) * sin(roll));

	double r31 = -sin(pitch);
	double r32 = cos(pitch) * sin(roll);
	double r33 = cos(pitch) * cos(roll);
	
	_cam_virtual_extrinsic = (Mat1f(4, 4) << 
		r11, r12, r13, (float)_cam_setting_x / 1000,

		r21, r22, r23, (float)_cam_setting_y / 1000,

		r31, r32, r33, (float)_cam_setting_z / 1000,

		0, 0, 0, 1
	);

	//
}

//transform an individual Mat point in 3d space to point on a 2d image  - added by Seamus Finlayson
void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	//std::cout << "extrinsic: " << _cam_virtual_extrinsic << std::endl;
	//std::cout << "intrinsic: " << _cam_virtual_intrinsic << std::endl;

	Mat pers_3d = _cam_virtual_intrinsic * _cam_virtual_extrinsic * pt3d_mat;
	//std::cout << pers_3d <<std::endl;

	pt.x = pers_3d.at<float>(0, 0) / pers_3d.at<float>(2, 0);
	pt.y = pers_3d.at<float>(1, 0) / pers_3d.at<float>(2, 0);
	//std::cout << pt << std::endl;
}

//transform a vector of Mat points in 3d space to a vector of points on a 2d image  - added by Seamus Finlayson
void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	Point2f pt;

	for (int i = 0; i < pts3d_mat.size(); i++) {

		Mat pers_3d = _cam_virtual_intrinsic * _cam_virtual_extrinsic * pts3d_mat.at(i);
		pt.x = pers_3d.at<float>(0, 0) / pers_3d.at<float>(2, 0);
		pt.y = pers_3d.at<float>(1, 0) / pers_3d.at<float>(2, 0);
		pts2d.push_back(pt);
	}
}

void CCamera::update_settings(Mat& im)
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

	calculate_intrinsic();
	calculate_extrinsic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lab 4
////////////////////////////////////////////////////////////////////////////////////////////////////////

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

	//in metres
	float size_aruco_square = 3.08/1000; // MEASURE THESE
	float size_aruco_mark = 1.86/1000; // MEASURE THESE

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
		if (key == ESC) break;
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
		if (key == ESC) break;
	}
}

//starts real cam and sets size  - added by Seamus Finlayson
void CCamera::init_real_cam(int cam_id) {

	//setup video input
	_inputVideo.open(cam_id);

	//set video dimensions
	_inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	_inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
}

//gets an image from the video stream - added by Seamus Finlayson
void CCamera::get_camera_image(Mat& frame) {

	//_inputVideo.grab();
	//Mat im;

	//// Get image
	//_inputVideo.retrieve(im);
	//im.copyTo(frame);



	//check if video is open
	if (_inputVideo.isOpened() == true) {

		//get frame from video
		_inputVideo >> frame;
	}

	//imshow("direct camera stream", frame); //works
}

//finds the pose of the charuco board if it is in the image passed to it - added by Seamus Finlayson
//based on code from https://docs.opencv.org/4.5.0/df/d4a/tutorial_charuco_detection.html
void CCamera::find_charuco_pose(Mat& image) {

	Mat draw_image;

	//make a copy in the image to draw on so markups dont interfere with image processing
	image.copyTo(draw_image);

	//set up board, dictionary, and params
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
	cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

	//get marker ids
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f> > markerCorners;
	cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

	// if at least one marker detected
	if (markerIds.size() > 0) {

		//draw markers on image
		cv::aruco::drawDetectedMarkers(draw_image, markerCorners, markerIds);
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> charucoIds;

		//find corners
		cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, _cam_real_intrinsic, _cam_real_dist_coeff);

		// if at least one charuco corner detected
		if (charucoIds.size() > 0) {
			cv::Scalar color = cv::Scalar(255, 0, 0);
			//cv::aruco::drawDetectedCornersCharuco(draw_image, charucoCorners, charucoIds, color);

			// cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec);
			bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, _cam_real_intrinsic, _cam_real_dist_coeff, _rvec, _tvec);

			// if charuco pose is valid draw origin
			if (valid) {
				//cv::aruco::drawAxis(image, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec, 0.1f);

				//update camera variables
				//m to mm
				_cam_setting_x = _tvec[0] * 1000;
				_cam_setting_y = _tvec[1] * 1000;
				_cam_setting_z = _tvec[2] * 1000;

				//delete this
				//rad to degrees
				//_cam_setting_roll = rvec[2] * 180;
				//_cam_setting_pitch = rvec[1] * 180;
				//_cam_setting_yaw = rvec[0] * 180;

				//copies aruco codes to display image
				draw_image.copyTo(image);
			}
		}
	}
}

//like transform_to_image but uses vectors created by open cv's estimatePoseCharucoBoard() function  - added by Seamus Finlayson
//must be used with real camera for lab 4 and beyond
void CCamera::transform_to_image_realcam(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d) {

	std::vector<Point3f> point_pts;
	Point3f point;

	for (int i = 0; i < pts3d_mat.size(); i++) {
		point.x = pts3d_mat.at(i).at<float>(0, 0);
		point.y = pts3d_mat.at(i).at<float>(1, 0);
		point.z = pts3d_mat.at(i).at<float>(2, 0);
		point_pts.push_back(point);
	}

	projectPoints(point_pts, _rvec, _tvec, _cam_real_intrinsic, _cam_real_dist_coeff, pts2d);
}