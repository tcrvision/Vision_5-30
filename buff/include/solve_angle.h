#pragma once

#pragma once
#include <opencv2/opencv.hpp>
#include "base.h"

using namespace cv;
using namespace std;
class SolveAngle
{
public:
    SolveAngle() {}
    SolveAngle(const char* file_path, float c_x, float c_y, float c_z, float barrel_y);
    // 能量机关角度解算

    void getBuffAngle(bool flag, vector<Point2f>& image_point, float ballet_speed, float buff_angle, float pre_angle, float gimbal_pitch, float& angle_x, float& angle_y, float& dist);
    float getBuffPitch(float dist, float tvec_y, float ballet_speed);


    void getAngle_ICRA(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float& dist);
    float BulletModel_ICRA(float x, float v, float angle);
    float GetPitch_ICRA(float x, float y, float v);
    void getAngle(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist);
    void Generate3DPoints(uint mode, Point2f offset_point);
    Mat cameraMatrix, distCoeffs;
    Mat object_point_mat;
    //vector<Point3f> objectPoints
    //    = vector<Point3f>{
    //cv::Point3f(-300, -300, 0),	//tl
    //cv::Point3f(300, -300, 0),	//tr
    //cv::Point3f(300, 300, 0),	//br
    //cv::Point3f(-300, 300, 0)	//bl
    vector<Point3f> objectPoints;


    vector<Point2f> projectedPoints;
    vector<Point2f> imagePoints;
    Mat rvec;
    Mat tvec;
    float height_world = 60.0;
    float overlap_dist = 100000.0;
    float barrel_ptz_offset_x = -0;
    float barrel_ptz_offset_y = -0; // mm   + ptz is up barrel

    float ptz_camera_x = 0;       // +left
    float ptz_camera_y = 52.5;       // + camera is  ptz
    float ptz_camera_z = -135;//-225;     // - camera is front ptz
    float scale = 0.99f;              // is calc distance scale not use pnp ,test

    int f_ = 1500;

public:
    float buff_h;

};

