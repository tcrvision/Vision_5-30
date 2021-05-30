//
// Created by czh on 2021/4/15.
//
#include<opencv2/opencv.hpp>
#include"base.h"

using namespace cv;
using namespace std;

#include "GxIAPI.h"
#include "DxImageProc.h"

class CameraDevice
{
public:
    CameraDevice();
    ~CameraDevice();
    int init();
    void getImage(Mat &img);
    uint64_t getFrameNumber();
private:
    GX_STATUS status;
    GX_DEV_HANDLE hDevice;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    Mat src;
    uint64_t  nFrameNum;
};