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
	//set real_cam to -100 to skip booting of real cam 
	//(it takes much longer to start up so it is best to skip this if you are not using the real camera)
	CRobot(int real_cam);
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;
	
	////////////////////////////////////
	// LAB 3

	vector<vector<Mat>> _simple_robot;

	CCamera _virtualcam;

	//CuArm uarm;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);
	
	////////////////////////////////////
	// LAB 4

	double _time, _time_old, _time_change; //track time in seconds
	double _angle_z; //set robot roation about the z axis
	void update();

	////////////////////////////////////
	// LAB 4
	
	int _do_animate; // Animation state machine
	void update_scara_settings(Mat& im); // update cvui scara control window
	void init();
	//integer versions for cvui sliders
	int _J0_pitch; //degrees
	int _J1_pitch; //degrees
	int _J2_roll; //degrees
	int _J3_x; //mm
	//float versions to eliminate rounding errors in sensitive data tracking
	double _J0_p; //degrees
	double _J1_p; //degrees
	double _J2_r; //degrees
	double _J3_xf; //mm
	double overshoot_correct(double target_value, double current_value, double full_range = 1000);
	Mat _end_effector_pose;

	////////////////////////////////////
	// LAB 6

	int _x, _y, _z, _theta;
	bool _do_startup;
	double _path_progress; //0 to 1
	Mat _warped_charuco;
	bool _track_marker;
	

public:

	/////////////////////////////
	// Lab 3

	Mat createHT(Vec3d t, Vec3d r);
	void create_simple_robot();
	void draw_simple_robot();

	/////////////////////////////
	// Lab 4

	void drawCoord_realcam(Mat& im, std::vector<Mat> coord3d);
	void drawBox_realcam(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void create_simple_robot_realcam();
	void draw_robot_on_image();

	/////////////////////////////
	// Lab 5

	void create_scara_robot();
	void draw_scara_robot();
	Mat fkin();
	void create_scara_robot_realcam();
	void draw_scara_robot_realcam();

	/////////////////////////////
	// Lab 6

	void draw_lab6A();
	void update_scara_settings_lab6A(Mat& im);
	void init_lab6A();
	void ikine(float x, float y, float z, float roll, float pitch, float yaw); //in mm and degrees

	void draw_lab6B();
	void warp_charuco_to_virtual(Mat charuco_image, Mat& charuco_in_virtual);
	void aruco_endeffector(Mat inputImage, int& x, int& z, int& theta);
}; 

