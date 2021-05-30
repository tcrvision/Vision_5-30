//
// Created by czh on 2021/5/5.
//
#include <cstring>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <additions.h>
#include <camera/camera_wrapper.h>
#include <log.h>
using namespace std;
using namespace cv;

extern WrapperHead *video;

bool checkReconnect(bool is_camera_connect) {
    if (!is_camera_connect) {
        int curr_gain = ((CameraWrapper* )video)->gain;
        int curr_exposure = ((CameraWrapper* )video)->exposure;
        delete video;
        video = new CameraWrapper(curr_exposure, curr_gain, 0/*, "armor"*/);
        is_camera_connect = video->init();
    }
    return is_camera_connect;
}