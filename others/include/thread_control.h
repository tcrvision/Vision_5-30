
#include<opencv2/opencv.hpp>
#include<iostream>
#include"thread"
#include"chrono"
#include"fstream"
#include "unistd.h"
#include "buff_detect.h"
#include"base.h"
#include"solve_angle.h"
#include"camera/camera_wrapper.h"
#include "camera_device.h"
#include "camera/wrapper_head.h"
#include "log.h"
#include "Armor_acquire.h"
#include "camera/save_video.h"
using namespace cv;
using namespace std;

void limit_angle(float& a, float max);
void protectData(int &a, int& b, int& c, int& d, int& e, int& f);
class ThreadControl {
public:
    ThreadControl();
    void ImageProcess();
    void ImageProduce();
    void GetData();
    void ImageWrite();
private:
    Mat image_;
    OtherParam other_param;
    int last_mode = 0;

    bool end_thread_flag = false;
    bool debug_enable_flag = false;
};