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
	double r11 = cos(r[0]) * cos(r[1]);
	double r12 = cos(r[0]) * sin(r[1]) * sin(r[2]) - sin(r[0]) * cos(r[2]);
	double r13 = cos(r[0]) * sin(r[1]) * cos(r[2]) + sin(r[0]) * cos(r[2]);
	double r21 = sin(r[0]) * cos(r[1]);
	double r22 = sin(r[0]) * sin(r[1]) * sin(r[2]) + cos(r[0]) * cos(r[2]);
	double r23 = sin(r[0]) * sin(r[1]) * cos(r[2]) + cos(r[0]) * sin(r[2]);
	double r31 = -sin(r[1]);
	double r32 = cos(r[1]) * sin(r[2]);
	double r33 = cos(r[1]) * cos(r[2]);


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
		std::cout << "before: " << points.at(i) << std::endl;
		points.at(i) = T * points.at(i);
		std::cout << "after: " << points.at(i) << std::endl;
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
	//CRobot::createBox(float w, float h, float d);
	//CRobot::drawBox(Mat & im, std::vector<Mat> box3d, Scalar colour);
}

void CRobot::draw_simple_robot()
{
	//clear canvas
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw coordinates
		std::vector<Mat> Origin = createCoord();
		//std::cout << "O before: \n" << Origin.at(0) << std::endl;
		
		Vec3d t(0, 0, 0);
		Vec3d r(0, 0, 0);

		Mat T = CRobot::createHT(t, r);
		std::cout << "tranform matrix: \n" << T << std::endl;
		//std::cout << "t: \n" << t << std::endl;
		CRobot::transformPoints(Origin, T);
		std::cout << "O after: \n" << Origin.at(1) << std::endl;

		CRobot::drawCoord(_canvas, Origin);

	//draw box
		//std::vector<Mat> box0 = CRobot::createBox(30,30,30);

		//Vec3d t(30, 30, 30);
		//Vec3d r(0, 0, 0);

		//Mat T = CRobot::createHT(t, r);
		//CRobot::transformPoints(box0, T);
		////std::cout << "box0 after: \n" << box0.at(0) << std::endl;

		//CRobot::drawBox(_canvas, box0, CV_RGB(0, 0, 255));

	//show sliders
	//_virtualcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}