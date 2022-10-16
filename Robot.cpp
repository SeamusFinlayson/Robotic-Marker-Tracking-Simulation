#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

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

	r[0] = r[0] / 57.2957795131; //convert from degrees to radians
	r[1] = r[1] / 57.2957795131; //convert from degrees to radians
	r[2] = r[2] / 57.2957795131; //convert from degrees to radians

	//need to fix
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

void CRobot::draw_simple_robot()
{
	//clear canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw coordinates
	std::vector<Mat> Origin = createCoord();
	drawCoord(_canvas, Origin);

	//draw robot
	create_simple_robot();

	//show sliders
	_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}