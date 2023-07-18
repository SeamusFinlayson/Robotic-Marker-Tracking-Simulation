#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CRobot::CRobot(int real_cam)
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	_warped_charuco = cv::Mat::zeros(_image_size, CV_8UC3);

	//start real camera - comment out to remove startup delay for lab 3,5ptA
	if (real_cam != -1)
		_virtualcam.init_real_cam(real_cam);

	//initialize time tracking
	_time_old = getTickCount() / getTickFrequency();
	_angle_z = 0;

	_do_startup = true;
	init();
}

CRobot::~CRobot()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB3
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create Homogeneous Transformation Matrix  - added by Seamus Finlayson
// takes input in degrees and metres
// returns transform matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{

	r[0] = r[0] / 57.2957795131; //convert from degrees to radians
	r[1] = r[1] / 57.2957795131; //convert from degrees to radians
	r[2] = r[2] / 57.2957795131; //convert from degrees to radians

	double r11 = cos(r[2]) * cos(r[1]);
	double r12 = cos(r[2]) * sin(r[1]) * sin(r[0]) - sin(r[2]) * cos(r[0]);
	double r13 = cos(r[2]) * sin(r[1]) * cos(r[0]) + sin(r[2]) * sin(r[0]);

	double r21 = sin(r[2]) * cos(r[1]);
	double r22 = sin(r[2]) * sin(r[1]) * sin(r[0]) + cos(r[2]) * cos(r[0]);
	double r23 = sin(r[2]) * sin(r[1]) * cos(r[0]) - cos(r[2]) * sin(r[0]);

	double r31 = -sin(r[1]);
	double r32 = cos(r[1]) * sin(r[0]);
	double r33 = cos(r[1]) * cos(r[0]);

	return (Mat1f(4, 4) << 
		r11, r12, r13, t[0], 

		r21, r22, r23, t[1], 

		r31, r32, r33, t[2], 

		0,   0,   0,   1
		);
}

//set box dimensions in metres
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
	//std::cout << "trandform matrix: " << T << std::endl;

	for (int i = 0; i < points.size(); i++)
	{
		//std::cout << "before: " << points.at(i) << std::endl;
		points.at(i) = T * points.at(i);
		//std::cout << "after: " << points.at(i) << std::endl;
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

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

//creates a "robot" in 3d space then converts it to a 2d image - added by Seamus Finlayson
void CRobot::create_simple_robot()
{
	//draw box for feet
	std::vector<Mat> feet = createBox(0.05, 0.05, 0.05);
	drawBox(_canvas, feet, CV_RGB(255, 0, 0));

	//draw box for legs
	std::vector<Mat> legs = createBox(0.05, 0.05, 0.05);
	Vec3d t(0, 0.05, 0);
	Vec3d r(0, 0, 0);
	Mat legs_trans = createHT(t, r);
	transformPoints(legs, legs_trans);
	drawBox(_canvas, legs, CV_RGB(255, 0, 0));

	//draw box for head
	std::vector<Mat> head = createBox(0.05, 0.05, 0.05);
	t = { 0, 0.15, 0 };
	Mat head_trans = createHT(t, r);
	transformPoints(head, head_trans);
	drawBox(_canvas, head, CV_RGB(255, 0, 0));

	//draw box for arm in positive x direction
	std::vector<Mat> arm_px = createBox(0.05, 0.05, 0.05);
	t = { 0.05, 0.10, 0 };
	Mat arm_px_trans = createHT(t, r);
	transformPoints(arm_px, arm_px_trans);
	drawBox(_canvas, arm_px, CV_RGB(0, 255, 0));

	//draw box for arm in negative x direction
	std::vector<Mat> arm_nx = createBox(0.05, 0.05, 0.05);
	t = { -0.05, 0.10, 0 };
	Mat arm_nx_trans = createHT(t, r);
	transformPoints(arm_nx, arm_nx_trans);
	drawBox(_canvas, arm_nx, CV_RGB(0, 0, 255));
}

//draws the "robot" created in create_simple_robot - added by Seamus Finlayson
void CRobot::draw_simple_robot()
{
	//clear canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw coordinatesasd
	std::vector<Mat> Origin = createCoord();
	drawCoord(_canvas, Origin);

	//draw robot
	create_simple_robot();

	//show sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB4
////////////////////////////////////////////////////////////////////////////////////////////////////////

//similar to drawCoord but uses rotation and translation determined from a real image - added by Seamus Finlayson
void CRobot::drawCoord_realcam(Mat& im, std::vector<Mat> coord3d) {
	//Point2f O, X, Y, Z;
	std::vector<Point2f> pts;

	_virtualcam.transform_to_image_realcam(coord3d, pts);

	line(im, pts.at(0), pts.at(1), CV_RGB(255, 0, 0), 1); // X=RED
	line(im, pts.at(0), pts.at(2), CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, pts.at(0), pts.at(3), CV_RGB(0, 0, 255), 1); // Z=BLUE
}

//similar to drawBox but uses rotation and translation determined from a real image - added by Seamus Finlayson
void CRobot::drawBox_realcam(Mat& im, std::vector<Mat> box3d, Scalar colour) {
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image_realcam(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

//creates a "robot" in 3d space then converts it to a 2d image for a camera image - added by Seamus Finlayson
void CRobot::create_simple_robot_realcam() {

	//set rotation for entire robot
	Vec3d r_all(0, 0, _angle_z);
	Vec3d t_all(0, 0, 0);
	Mat trans_all = createHT(t_all, r_all); //perform after others

	//draw box for feet
	std::vector<Mat> feet = createBox(0.05, 0.05, 0.05);
	Vec3d t(0, 0, 0.025);
	Vec3d r(0, 0, 0);
	Mat feet_trans = createHT(t, r);
	transformPoints(feet, feet_trans);
	transformPoints(feet, trans_all);
	drawBox_realcam(_canvas, feet, CV_RGB(255, 0, 0));

	//draw box for legs
	std::vector<Mat> legs = createBox(0.05, 0.05, 0.05);
	t = { 0, 0, 0.1-0.025};
	Mat legs_trans = createHT(t, r);
	transformPoints(legs, legs_trans);
	transformPoints(legs, trans_all);
	drawBox_realcam(_canvas, legs, CV_RGB(255, 0, 0));

	//draw box for head
	std::vector<Mat> head = createBox(0.05, 0.05, 0.05);
	t = {0, 0, 0.2-0.025};
	Mat head_trans = createHT(t, r);
	transformPoints(head, head_trans);
	transformPoints(head, trans_all);
	drawBox_realcam(_canvas, head, CV_RGB(255, 0, 0));

	//draw box for arm in positive x direction
	std::vector<Mat> arm_py = createBox(0.05, 0.05, 0.05);
	t = {0, 0.05, 0.15-0.025};
	Mat arm_px_trans = createHT(t, r);
	transformPoints(arm_py, arm_px_trans);
	transformPoints(arm_py, trans_all);
	drawBox_realcam(_canvas, arm_py, CV_RGB(0, 255, 0));

	//draw box for arm in negative x direction
	std::vector<Mat> arm_ny = createBox(0.05, 0.05, 0.05);
	t = {0, -0.05, 0.15-0.025};
	Mat arm_nx_trans = createHT(t, r);
	transformPoints(arm_ny, arm_nx_trans);
	transformPoints(arm_ny, trans_all);
	drawBox_realcam(_canvas, arm_ny, CV_RGB(0, 0, 255));
}

//draws a robot on a camera image based on coordintes extrapolated from the image - added by Seamus Finlayson
void CRobot::draw_robot_on_image() {

	//update robot parameters
	update();

	//get new image
	_virtualcam.get_camera_image(_canvas);

	//get pose of charuco board
	_virtualcam.find_charuco_pose(_canvas);

	//draw coordinatesasd
	std::vector<Mat> Origin = createCoord();
	drawCoord_realcam(_canvas, Origin);

	//create robot
	create_simple_robot_realcam();

	//show sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

//updates robot variables and time - added by Seamus Finlayson
void CRobot::update() {

	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;

	//update robot parameters
	float angular_velocity_z = 20; //degrees/s
	_angle_z += angular_velocity_z * _time_change; //degrees/s * time in s
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5A
////////////////////////////////////////////////////////////////////////////////////////////////////////

//creates a scara robot in 3d space then converts it to a 2d image - added by Seamus Finlayson
void CRobot::create_scara_robot() {

	vector<vector<Mat>> joint_points; //points that show the pose of each joint
	std::vector<Mat> joint_point; //a single set of points that show the pose of a joint

	//world origin coordinate
	Mat W = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
	std::vector<Mat> Origin = createCoord(); //is not transformed

	//joint 1 - on top of platform, pitch is free
	Mat J0_W = W * createHT(Vec3d(0, 0.15, 0), Vec3d(0, _J0_pitch, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J0_W);
	joint_points.push_back(joint_point);

	//joint 2 - end of first link, arm 1, pitch is free
	Mat J1_W = J0_W * createHT(Vec3d(0.15, 0, 0), Vec3d(0, _J1_pitch, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J1_W);
	joint_points.push_back(joint_point);

	//joint 3 - end of second link, arm 2, roll is free
	Mat J2_W = J1_W * createHT(Vec3d(0.15, 0, 0), Vec3d(_J2_roll, 0, -90));
	joint_point = createCoord();
	transformPoints(joint_point, J2_W);
	joint_points.push_back(joint_point);

	//joint 4 - end of third link, lifter, x is free, end effector
	Mat J3_W = J2_W * createHT(Vec3d((float)_J3_x / 1000, 0, 0), Vec3d(0, 0, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J3_W);
	joint_points.push_back(joint_point);

	//put coordinates in vector - not implemented
	//vector<Mat> joint_W; //all joints with respect to the world coordinate system

	_end_effector_pose = J3_W;

	//draw world
	drawCoord(_canvas, Origin);

	//draw coordinate axis
	for (std::vector<Mat> joint_point : joint_points) {
		drawCoord(_canvas, joint_point);
	}

	//link 1, platform
	std::vector<Mat> platform = createBox(0.05, 0.15 - 0.05/2, 0.05);
	Mat platform_offset = createHT(Vec3d(0, (0.15 - 0.05/2) / 2, 0), Vec3d(0, 0, 0));
	transformPoints(platform, platform_offset); //offset to place endpoints correctly
	transformPoints(platform, W); //place in the correct coordinate
	drawBox(_canvas, platform, CV_RGB(255, 255, 255));

	//link 2, arm 1
	std::vector<Mat> arm1 = createBox(0.15, 0.05, 0.05);
	Mat arm1_offset = createHT(Vec3d(0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(arm1, arm1_offset); //offset to place endpoints correctly
	transformPoints(arm1, J0_W); //place in the correct coordinate
	drawBox(_canvas, arm1, CV_RGB(255, 0, 0));

	//link 3, arm 2
	std::vector<Mat> arm2 = createBox(0.15, 0.05, 0.05);
	Mat arm2_offset = createHT(Vec3d(0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(arm2, arm2_offset); //offset to place endpoints correctly
	transformPoints(arm2, J1_W); //place in the correct coordinate
	drawBox(_canvas, arm2, CV_RGB(0, 255, 0));

	//link 4, lifter
	std::vector<Mat> lifter = createBox(0.15, 0.025, 0.025);
	Mat lifter_offset = createHT(Vec3d(-0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(lifter, lifter_offset); //offset to place endpoints correctly
	transformPoints(lifter, J3_W); //place in the correct coordinate
	drawBox(_canvas, lifter, CV_RGB(0, 0, 255));
}

//draws the scara robot on a blank canvas - added by Seamus Finlayson
void CRobot::draw_scara_robot() {
	//clear canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw origin
	std::vector<Mat> Origin = createCoord();
	drawCoord(_canvas, Origin);

	//show scara control sliders
	update_scara_settings(_canvas);

	//create and draw robot
	create_scara_robot();

	//show viewing pose sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

//draw a cvui window on the canvas to manipulate scara - added by Seamus Finlayson
void CRobot::update_scara_settings(Mat& im) {
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 30;

	//button to start state machine
	if (cvui::button(im, _setting_window.x, _setting_window.y, 92, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	//button to reset all jopint angles to default 0 positions
	if (cvui::button(im, _setting_window.x + 97, _setting_window.y, 92, 30, "Reset"))
	{
		init();
	}

	//draw and get data from J0 control bar
	_setting_window.y += 45;
	if (cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J0_pitch, -180, 180)) {
		_J0_p = _J0_pitch; //not used but this is how to imediatly use float values
	}
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J0");

	//draw and get data from J1 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J1_pitch, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J1");

	//draw and get data from J2 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J2_roll, -360, 360);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J2");

	//draw and get data from J3 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J3_x, 0, 150);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J3");

	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;

	//set angular velocity for all animations
	double std_angular_velocity = 360; //degrees/s
	double std_velocity = 200; //mm/s

	//state machine to show robot range of motion
	if (_do_animate != 0)
	{
		if (_do_animate == 1)
		{
			//state 1: rotate joint 0 from 0 to 180
			_J0_p += overshoot_correct(180, _J0_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J0_pitch = _J0_p;

			if (_J0_p >= 180) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2: rotate joint 0 from 180 to -180
			_J0_p -= overshoot_correct(-180, _J0_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J0_pitch = _J0_p;

			if (_J0_p <= -180) { _do_animate = 3; }
		}
		else if (_do_animate == 3)
		{
			// state 3: rotate joint 0 from -180 to 0
			_J0_p += overshoot_correct(0, _J0_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J0_pitch = _J0_p;

			if (_J0_p >= 0) { _do_animate = 4; }
		}
		if (_do_animate == 4)
		{
			// state 4: rotate joint 1 from 0 to 180
			_J1_p += overshoot_correct(180, _J1_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J1_pitch = (int)_J1_p;

			if (_J1_p >= 180) { _do_animate = 5; }
		}
		else if (_do_animate == 5)
		{
			// state 5: rotate joint 1 from 180 to -180
			_J1_p -= overshoot_correct(-180, _J1_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J1_pitch = _J1_p;

			if (_J1_p <= -180) { _do_animate = 6; }
		}
		else if (_do_animate == 6)
		{
			// state 6: rotate joint 1 from -180 to 0
			_J1_p += overshoot_correct(0, _J1_p) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J1_pitch = _J1_p;

			if (_J1_p >= 0) { _do_animate = 7; }
		}
		else if (_do_animate == 7)
		{
			// state 7: rotate joint 0 to 360
			_J2_r += overshoot_correct(360, _J2_r) * std_angular_velocity * _time_change; //degrees/s * time in s
			_J2_roll = _J2_r;

			if (_J2_r >= 360) { _do_animate = 8; }
		}
		else if (_do_animate == 8)
		{
			// state 8: move joint 3 from 15 to 0
			_J3_xf += overshoot_correct(150, _J3_xf) * std_velocity * _time_change; //mm/s * time in s
			_J3_x = _J3_xf;

			if (_J3_xf >= 150) { _do_animate = 9; }
		}
		else if (_do_animate == 9)
		{
			// state 9: rotate joint 0 to 360
			_J3_xf -= overshoot_correct(0, _J3_xf) * std_velocity * _time_change; //mm/s * time in s
			_J3_x = _J3_xf;

			if (_J3_xf <= 0) { _do_animate = 10; }
		}
		else if (_do_animate == 10) {
			//loops animation
			init();
			_do_animate = 1;
			std::cout << "animation done" << std::endl;
		}
	}

	//cvui::update(); // should only be called one for buttons to work, it is already called for the Ccamera menu
}

//initial or reset animation adn robot position variables - added by Seamus Finlayson
void CRobot::init() {

	// reset variables
	_do_animate = 0;
	_J0_pitch = 0;
	_J1_pitch = 0;
	_J2_roll = 0;
	_J3_x = 0;
	_J0_p = 0;
	_J1_p = 0;
	_J2_r = 0;
	_J3_xf = 0;
	_end_effector_pose = createHT(Vec3f(0, 0, 0), Vec3f(0, 0, 0));
}

//uses distance from target value to non linearly decrease velocity - added by Seamus Finlayson
double CRobot::overshoot_correct(double target_value, double current_value, double full_range) {

	double coef;
	//if (target_value != 0) {

	//	coef = pow(abs((target_value - current_value + 0.1) / target_value) * 10, 0.7);
	//}
	//else {
		coef = pow(abs((target_value - current_value + 0.1) / full_range) * 10, 0.7);
	//}
	//std::cout << "coef: " << std::setw(10) << coef << ",\tdiff: " << std::setw(10) << abs(target_value - current_value) << std::endl;

	//if coefficient slows velocity
	if (coef < 1) {

		//reduce velocity according to damping curve
		return coef;
	}
	else {

		//do not reduce velocity
		return 1;
	}
}

//returns end effector pose - added by Seamus Finlayson
Mat CRobot::fkin() {

	//for use in inverse kinematics
	Mat W = createHT(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
	Mat J0_W = W * createHT(Vec3d(0, 0.15, 0), Vec3d(0, _J0_pitch, 0));
	Mat J1_W = J0_W * createHT(Vec3d(0.15, 0, 0), Vec3d(0, _J1_pitch, 0));
	Mat J2_W = J1_W * createHT(Vec3d(0.15, 0, 0), Vec3d(_J2_roll, 0, -90));
	Mat end_effector_pose = J2_W * createHT(Vec3d(0.15 - (float)_J3_x / 1000, 0, 0), Vec3d(0, 0, 0));

	_x = end_effector_pose.at<float>(0, 3) * 1E3;
	_y = end_effector_pose.at<float>(1, 3) * 1E3;
	_z = -1 * end_effector_pose.at<float>(2, 3) * 1E3;
	std::cout << "x: " << std::setw(5) << _x << " y: " << std::setw(5) << _y << " z: " << std::setw(5) << _z << std::endl;

	//double r11 = end_effector_pose.at<double>(0, 0);
	//double r21 = end_effector_pose.at<double>(1, 0);
	//double r31 = end_effector_pose.at<double>(2, 0);
	//_J2_roll = atan2(-1 * r31, sqrt(pow(r11, 2) + pow(r21, 2))) * 180 / 3.14159;
	//double roll = atan2(-1 * r31, sqrt(pow(r11, 2) + pow(r21, 2))) * 180 / 3.14159;
	//std::cout << "r11: " << std::setw(5) << r11 << " r21: " << std::setw(5) << r21 << " r31: " << std::setw(5) << r31 << " theta: " << std::setw(5) << roll <<  std::endl;

	//for numerical method inverse kinematics
	return end_effector_pose;

	////for use as get function
	//return _end_effector_pose;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5B
////////////////////////////////////////////////////////////////////////////////////////////////////////

//creates a scara robot in 3d space then converts it to a 2d image - added by Seamus Finlayson
void CRobot::create_scara_robot_realcam() {

	vector<vector<Mat>> joint_points; //points that show the pose of each joint
	std::vector<Mat> joint_point; //a single set of points that show the pose of a joint

	//world origin coordinate
	Mat W = createHT(Vec3d(0, 0, 0), Vec3d(90, 0, 90)); //rotations flip the robot to the upright position
	std::vector<Mat> Origin = createCoord();
	transformPoints(Origin, W);

	//joint 1 - on top of platform, pitch is free
	Mat J0_W = W * createHT(Vec3d(0, 0.15, 0), Vec3d(0, _J0_pitch, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J0_W);
	joint_points.push_back(joint_point);

	//joint 2 - end of first link, arm 1, pitch is free
	Mat J1_W = J0_W * createHT(Vec3d(0.15, 0, 0), Vec3d(0, _J1_pitch, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J1_W);
	joint_points.push_back(joint_point);

	//joint 3 - end of second link, arm 2, roll is free
	Mat J2_W = J1_W * createHT(Vec3d(0.15, 0, 0), Vec3d(_J2_roll, 0, -90));
	joint_point = createCoord();
	transformPoints(joint_point, J2_W);
	joint_points.push_back(joint_point);

	//joint 4 - end of third link, lifter, x is free, end effector
	Mat J3_W = J2_W * createHT(Vec3d((float)_J3_x / 1000, 0, 0), Vec3d(0, 0, 0));
	joint_point = createCoord();
	transformPoints(joint_point, J3_W);
	joint_points.push_back(joint_point);

	//put coordinates in vector - not implemented
	//vector<Mat> joint_W; //all joints with respect to the world coordinate system

	_end_effector_pose = J3_W;

	//draw world
	drawCoord_realcam(_canvas, Origin);

	//draw coordinate axis
	for (std::vector<Mat> joint_point : joint_points) {
		drawCoord_realcam(_canvas, joint_point);
	}

	//link 1, platform
	std::vector<Mat> platform = createBox(0.03, 0.15 - 0.03 / 2, 0.03);
	Mat platform_offset = createHT(Vec3d(0, (0.15 - 0.05 / 2) / 2, 0), Vec3d(0, 0, 0));
	transformPoints(platform, platform_offset); //offset to place endpoints correctly
	transformPoints(platform, W); //place in the correct coordinate
	drawBox_realcam(_canvas, platform, CV_RGB(255, 255, 255));

	//link 2, arm 1
	std::vector<Mat> arm1 = createBox(0.15, 0.03, 0.03);
	Mat arm1_offset = createHT(Vec3d(0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(arm1, arm1_offset); //offset to place endpoints correctly
	transformPoints(arm1, J0_W); //place in the correct coordinate
	drawBox_realcam(_canvas, arm1, CV_RGB(255, 0, 0));

	//link 3, arm 2
	std::vector<Mat> arm2 = createBox(0.15, 0.03, 0.03);
	Mat arm2_offset = createHT(Vec3d(0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(arm2, arm2_offset); //offset to place endpoints correctly
	transformPoints(arm2, J1_W); //place in the correct coordinate
	drawBox_realcam(_canvas, arm2, CV_RGB(0, 255, 0));

	//link 4, lifter
	std::vector<Mat> lifter = createBox(0.15, 0.02, 0.02);
	Mat lifter_offset = createHT(Vec3d(-0.15 / 2, 0, 0), Vec3d(0, 0, 0));
	transformPoints(lifter, lifter_offset); //offset to place endpoints correctly
	transformPoints(lifter, J3_W); //place in the correct coordinate
	drawBox_realcam(_canvas, lifter, CV_RGB(0, 0, 255));
}

//draws the scara robot on a blank canvas - added by Seamus Finlayson
void CRobot::draw_scara_robot_realcam() {

	//get new image
	_virtualcam.get_camera_image(_canvas);

	////get pose of charuco board
	_virtualcam.find_charuco_pose(_canvas);

	//draw robot
	create_scara_robot_realcam();

	//show scara control sliders
	update_scara_settings(_canvas);

	//show viewing pose sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB6A
////////////////////////////////////////////////////////////////////////////////////////////////////////

//main loop for lab 6A, draws a scara robot in virtual space that can be controlled using invese kinematics - added by Seamus Finlayson
void CRobot::draw_lab6A() {

	//set variables to startup conditions
	if (_do_startup) {
		init_lab6A();
		_do_startup = false;
	}

	//clear canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw origin
	std::vector<Mat> Origin = createCoord();
	drawCoord(_canvas, Origin);

	//show scara control sliders
	update_scara_settings_lab6A(_canvas);

	ikine(_x, _y, _z, 0, _theta, 0);

	//create and draw robot
	create_scara_robot();

	//show viewing pose sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

//updates variables related to the lab 6 scara robot and animates a path for the end effector to travel - added by Seamus Finlayson
void CRobot::update_scara_settings_lab6A(Mat& im) {
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	//draw and get data from J0 control bar
	_setting_window.x += 5;
	_setting_window.y += 20;
	if (cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J0_pitch, -180, 180)) {
		_J0_p = _J0_pitch; //not used but this is how to imediatly use float values
	}
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J0");

	//draw and get data from J1 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J1_pitch, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J1");

	//draw and get data from J2 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J2_roll, -360, 360);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J2");

	//draw and get data from J3 control bar
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_J3_x, 0, 150);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J3");

	_setting_window.y += 55;

	//button to start state machine
	if (cvui::button(im, _setting_window.x, _setting_window.y, 92, 30, "Animate")) {
		init_lab6A();
		_do_animate = 1;
	}

	//button to reset all jopint angles to default 0 positions
	if (cvui::button(im, _setting_window.x + 97, _setting_window.y, 92, 30, "Reset")) {
		init_lab6A();
	}
	
	//x slider, y slider, z slider, theta slider added for lab 6a
	//ikine x slider
	_setting_window.y += 45;
	if (cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_x, -299, 299)) {
		if (sqrt(pow(_x, 2) + pow(_z, 2)) >= 300) {
			_z = sqrt(pow(299, 2) - pow(_x, 2)) * (_z / abs(_z));
		}
	}
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "X");

	//ikine z slider
	_setting_window.y += 45;
	if (cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_z, -299, 299)) {
		if (sqrt(pow(_x, 2) + pow(_z, 2)) >= 300) {
			_x = sqrt(pow(299, 2) - pow(_z, 2)) * (_x/abs(_x));
		}
	}
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Z");

	//ikine y slider
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_y, 0, 150);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Y");

	//ikine theta slider
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_theta, -360, 360);
	cvui::text(im, _setting_window.x + 170, _setting_window.y + 20, "theta");

	//do marker tracking
	_setting_window.y += 45;
	if (cvui::checkbox(im, _setting_window.x, _setting_window.y, "Track Marker", &_track_marker)) {
		aruco_endeffector(_warped_charuco, _x, _z, _theta);
		_y = 150;
	}

	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;

	//set angular velocity for all animations
	double std_velocity = 0.4; //mm/s along x axis

	//state machine to show robot range of motion
	if (_do_animate != 0)
	{
		if (_do_animate == 1)
		{
			//state 1: reset path progress
			_path_progress = 0;

			if (true) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2: move forward along path
			_path_progress += std_velocity * _time_change;
			_x = -150 + 400 * _path_progress;
			_z = -250 + 400 * _path_progress;

			if (_path_progress >= 1) { _do_animate = 3; }
		}
		else if (_do_animate == 3)
		{

			// state 3: move backwards along path
			_path_progress -= std_velocity * _time_change;
			_x = -150 + 400 * _path_progress;
			_z = -250 + 400 * _path_progress;

			if (_path_progress <= 0) { _do_animate = 5; }
		}
		else if (_do_animate == 5) {
			//loops animation
			_do_animate = 1;
			std::cout << "animation done" << std::endl;
		}
		std::cout << "path pregress: " << _path_progress << std::endl;
	}

	//cvui::update(); // should only be called one for buttons to work, it is already called for the Ccamera menu
}

//resets variables related to the scara robot - added by Seamus Finlayson
void CRobot::init_lab6A() {

	// reset variables
	_do_animate = false;
	_x = 200;
	_y = 0;
	_z = 20;
	_theta = 0;
	_track_marker = false;
}

//uses inverse kinematics to set robot joint angles so the end effector is placed at the desired pose - added by Seamus Finlayson
void CRobot::ikine(float x, float y, float z, float roll, float pitch, float yaw) {

	//scaling
	x = x / 1E3;
	z = z / 1E3;

	if (sqrt(pow(x, 2) + pow(z, 2)) < 0.3) {

		//main arms
		_J0_pitch = 2 * atan2(3 * z + sqrt(-100 * pow(x, 4) - 200 * pow(x, 2) * pow(z, 2) + 9 * pow(x, 2) - 100 * pow(z, 4) + 9 * pow(z, 2)), 10 * pow(x, 2) + 3 * x + 10 * pow(z, 2)) * 180 / 3.14159;
		_J1_pitch = -2 * atan2(sqrt(-100 * pow(x, 2) - 100 * pow(z, 2) + 9), (10 * sqrt(pow(x, 2) + pow(z, 2)))) * 180 / 3.14159;
		//std::cout << "J0 pitch: " << std::setw(10) << _J0_pitch << "\tJ1 pitch: " << std::setw(10) << _J1_pitch << std::endl;
		//std::cout << "ok: pose reachable" << std::endl;
	}
	else {
		std::cout << "error: pose not reachable" << std::endl;
	}
	//rotate lifter
	_J2_roll = 1 * (_J0_pitch + _J1_pitch) + pitch;

	//move lifter
	_J3_x = y;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB6B
////////////////////////////////////////////////////////////////////////////////////////////////////////

//main loop for lab 6B, draws a scara robot that can be controlled using invese kinematics - added by Seamus Finlayson
void CRobot::draw_lab6B() {

	//set variables to startup conditions
	if (_do_startup) {
		init_lab6A();
		_do_startup = false;
	}

	//get new video stream image from camera
	Mat camera_image;
	_virtualcam.get_camera_image(camera_image);
	cv::imshow("video stream", camera_image);

	//warp board in image to a rectangular image
	_virtualcam.warp_to_rectangle(camera_image, _warped_charuco);

	//place and warp board into virtual space
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60); //reset canvas
	warp_charuco_to_virtual(_warped_charuco, _canvas);

	//get pose of charuco board
	//_virtualcam.find_charuco_pose(_canvas);

	//show scara control sliders
	update_scara_settings_lab6A(_canvas);

	//update end effector pose
	ikine(_x, _y, _z, 0, _theta, 0);

	//create and draw robot
	create_scara_robot();

	//show viewing pose sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

//take the image of a charuco board with corner ids that was been warped into a rectangle and warp it into virtual space - added by Seamus Finlayson
void CRobot::warp_charuco_to_virtual(Mat charuco_image, Mat& charuco_in_virtual) {

	//check that image of charuco board has been obtained
	if (charuco_image.empty() == false) {

		//get corners of charuco board image
		vector<Point2f> source_corners;
		source_corners.push_back(Point2i(0, 0));
		source_corners.push_back(Point2i(0, charuco_image.rows));
		source_corners.push_back(Point2i(charuco_image.cols, 0));
		source_corners.push_back(Point2i(charuco_image.cols, charuco_image.rows));

		//create destination points in 3d (in metres)
		vector<Mat> destination_corners3d;
		destination_corners3d.push_back(Mat((Mat1f(4, 1) << 0, 0, 0, 1)));
		destination_corners3d.push_back(Mat((Mat1f(4, 1) << 0, 0, 0.169, 1)));
		destination_corners3d.push_back(Mat((Mat1f(4, 1) << 0.24, 0, 0, 1)));
		destination_corners3d.push_back(Mat((Mat1f(4, 1) << 0.24, 0, 0.169, 1)));

		//get destination corners on the 2d image, this is what is used to do the transform
		vector<Point2f> destination_corners2d;
		_virtualcam.transform_to_image(destination_corners3d, destination_corners2d); //transforms set of points in 3d to a 2d image

		//create warp trandformation matrix
		cv::Mat pTransMat = cv::getPerspectiveTransform(source_corners, destination_corners2d);

		//do warp
		warpPerspective(charuco_image, charuco_in_virtual, pTransMat, _image_size);

		//debug
		//imshow("charuco in virtual", charuco_in_virtual);
		//waitKey(10);
		//std::cout << "in virtual" << std::endl;
	}
	else {
		std::cout << "Warning: no image in warp_charuco_to_virtual()" << std::endl;
	}
}

//based on the image get the x,z, and theta end effector pose - added by Seamus Finlayson
void CRobot::aruco_endeffector(Mat inputImage, int& x, int& z, int& theta) {

	if (inputImage.empty() == false) {
		//find markers in input image
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

		std::vector<int>::iterator target_id_address;
		target_id_address = find(markerIds.begin(), markerIds.end(), 50); //find target id's address, for lab 6b this value is 50
		if (target_id_address != markerIds.end()) {	//make sure target id was in image

			//get index of target id
			int index;
			index = target_id_address - markerIds.begin();

			//find centre of target id
			Point2f centre(0, 0);
			for (Point2f corner : markerCorners.at(index)) {
				centre.x += corner.x;
				centre.y += corner.y;
			}
			//take scale and take the average (in mm)
			centre.x *= static_cast<float>(240) / inputImage.cols / 4;
			centre.y *= -1 * static_cast <float>(169) / inputImage.rows / 4;

			//output
			x = centre.x;
			z = centre.y;

			//find angle of target (in degrees)
			Point2f slope;
			slope.x = markerCorners.at(index).at(1).x - markerCorners.at(index).at(0).x;
			slope.y = markerCorners.at(index).at(1).y - markerCorners.at(index).at(0).y;
			theta = atan2(slope.y, slope.x) * 180 / 3.14159;
		}
	}
}