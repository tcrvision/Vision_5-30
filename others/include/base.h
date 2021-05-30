#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>

#include"serialport.h"

using namespace std;
using namespace cv;
#define USE_GALAXY  //使用大恒or华腾
#define BUFFER_SIZE 1
//是否进行Debug
#define Debug
#ifdef Debug
	#define ARMOR_TRACK_BAR //Armor
	#define BUFF_TRACK_BAR  //BUFF
	#define Auto
	#define DEBUG_BUFF_DETECT
	#define IMAGESHOW //Origin ImageShow
#endif
//线程开关
#define GET_DATA_THREAD  //串口
//#define SAVE_VIDEO_THREAD //保存视频
//
#define WAITKEY 1

#ifdef USE_GALAXY
    #define CAMERA_PATH "/home/tkkrobot/share/TCR-Windmill/cmake-build-debug/cameraParamsGalaxy6mm1.xml"
#endif
#ifndef USE_GALAXY
    #define CAMERA_PATH "/home/tkk-robort/workspace/TCR-Windmill/build/cameraParams8mmBubing1.xml"
#endif

#define BUFF_VIDEO_PATH "./V1.avi"  //视频路径
#define BUFF_OFFSET_x 101
#define BUFF_OFFSET_y 118
#define WORLD_OFFSET_X 750
#define COLOR_TH 50
//华腾相机参数
#define ENERGY_CAMERA_EXPOSURE  (15)
#define ENERGY_CAMERA_GAIN  (8)
//代码调试参数
//#define Predict
#define BUFF_H 800//800 :430
#define BUFF_DISTANCE 7000
#define BULLET_SPEED 28
#define RESET_ANGLE -10 // 1:-20 else: -10  // 复位绝对角度
#define REPEAT_FIRE_TIME 1000
//摄像头坐标系到云台坐标系
#define LONG_X 0.0f
#define LONG_Y 60.65f
#define LONG_Z -114.65f
#define PTZ_TO_BARREL -25.0f
//视频or相机
//#define DEBUG_VIDEO
#define OPEN_CAMERA

#define END_THREAD if(end_thread_flag) return;
#define TIME_START(a) double a=(double)getTickCount();
#define TIME_END(a) a = ((double)getTickCount() - a) / getTickFrequency();cout<<#a<<" "<<(double)(1.0/a)<<endl;
//串口路径
#define SERIAL_PATH "/dev/485_USB"
//串口波特率
#define SERIAL_BAUD B115200
#define INFO(a) cout<<#a<<"="<<a<<endl;
#define NOTICE(test, num){                   \
    static bool flag = true;            \
    static int i=0; \
    if(flag)                            \
{              \
    i++;                          \
    std::cout << test << std::endl; \
    if(i>=num)               \
    flag = false;                   \
    }                                   \
    }                                       \

struct OtherParam
{
    int color = 0; //我方车辆颜色，0是蓝色，1是红色。
    int mode = 0; //视觉模式，0是自瞄模式，1是能量机关模式
    float gimbal_data;
    float buff_offset_x;
    float buff_offset_y;
    int command = 0;
    int bulletspeed;
   
};
