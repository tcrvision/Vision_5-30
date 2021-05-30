//
// Created by czh on 2021/5/5.
//
#include<iostream>
#include<camera/camera_wrapper.h>
#include<systime.h>
#include<log.h>
using namespace std;
using namespace cv;

CameraWrapper::CameraWrapper(int exposure, int gain, int camera_mode, const std::string &n) :
        name(n),
        init_done(false),
        mode(camera_mode),
        camera_cnts(2),
        camera_status(-1),
        iplImage(nullptr),
        rgb_buffer(nullptr),
        channel(3),
        gain(gain),
        exposure(exposure){
}
void cameraCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext){
    CameraWrapper *c = (CameraWrapper*)pContext;
    CameraImageProcess(hCamera, pFrameBuffer, c->rgb_buffer, pFrameHead);
    auto iplImage = cvCreateImageHeader(cvSize(pFrameHead->iWidth, pFrameHead->iHeight), IPL_DEPTH_8U, c->channel);
    cvSetData(iplImage, c->rgb_buffer, pFrameHead->iWidth * c->channel);  //�˴�ֻ������ָ�룬��ͼ������ݿ��������赣��ת��Ч��
    c->src_queue.push(cv::cvarrToMat(iplImage).clone());
}
bool CameraWrapper::init() {
    CameraSdkInit(1);
    int camera_enumerate_device_status = CameraEnumerateDevice(camera_enum_list, &camera_cnts);
    if (camera_enumerate_device_status != CAMERA_STATUS_SUCCESS) {
        LOGE("CameraEnumerateDevice fail with %d!", camera_enumerate_device_status);
    }
    if (camera_cnts == 0) {
        LOGE("No camera device detected!");
        return false;
    } else if (camera_cnts >= 1) {
        LOGM("%d camera device detected!", camera_cnts);
    }
    int i;
    for (i = 0; i < camera_cnts; i++) {
        camera_status = CameraInit(&camera_enum_list[i], -1, -1, &h_camera);
        if (camera_status != CAMERA_STATUS_SUCCESS) {
            CameraUnInit(h_camera);
            continue;
        }
        CameraGetFriendlyName(h_camera, camera_name);
        if (name == "NULL" || strcmp(name.data(), camera_name) == 0) {
            break;
        }
        CameraUnInit(h_camera);
    }
    if (i >= camera_cnts) {
        LOGE("No device name %s or device open error!!", name.data());
        return false;
    }

    auto status = CameraGetCapability(h_camera, &tCapability);
    if (status != CAMERA_STATUS_SUCCESS) {
        cout << "CameraGetCapability return error code " << status << endl;
        return false;
    }

    rgb_buffer = (unsigned char *) malloc(tCapability.sResolutionRange.iHeightMax *
                                          tCapability.sResolutionRange.iWidthMax * 3);

    //CameraLoadParameter(h_camera, PARAMETER_TEAM_A);
    //CameraSetAeState(h_camera, false);
    CameraSetExposureTime(h_camera, 1500.0);
    CameraSetAnalogGain(h_camera, 12);
    CameraSetExtTrigShutterType(h_camera,EXT_TRIG_EXP_GRR); //�������Ϊȫ�ֿ��ŷ�ʽ
    CameraSetTriggerMode(h_camera,0);     //������������ɼ�ģʽ
//    //CameraSetAeTarget(hCamera,4);       //�ع�
//    CameraSetExposureTime(h_camera,1500.0);//�ع�ʱ�� ��λ΢��
//    CameraSetAnalogGain(h_camera,4);   //����
//    CameraSetGamma(h_camera,20); //Gamma
//    CameraSetContrast(h_camera,200); //�Աȶ�
//    CameraSetLutMode(h_camera,LUTMODE_PARAM_GEN);

    double t;
    int g;
    CameraGetExposureTime(h_camera, &t);
    CameraGetAnalogGain(h_camera, &g);
    LOGM("Exposure time: %lfms, gain:%d", t / 1000.0, g);
    /*��SDK���빤��ģʽ����ʼ��������������͵�ͼ��
    ���ݡ������ǰ����Ǵ���ģʽ������Ҫ���յ�
    ����֡�Ժ�Ż����ͼ��    */
    CameraPlay(h_camera);

    /*�����������������
    ���� CameraSetExposureTime   CameraGetExposureTime  ����/��ȡ�ع�ʱ��
         CameraSetImageResolution  CameraGetImageResolution ����/��ȡ�ֱ���
         CameraSetGamma��CameraSetContrast��CameraSetGain������ͼ��٤���Աȶȡ�RGB��������ȵȡ�
         CameraGetFriendlyName    CameraSetFriendlyName ��ȡ/����������ƣ������ƿ�д�����Ӳ����
    */
    cout << tCapability.sIspCapacity.bMonoSensor << endl;
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);
        LOGM("camera %s mono ", camera_name);
    } else {
        channel = 3;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
        LOGM("camera %s color ", camera_name);
    }
    if(mode == 2){
        CameraSetCallbackFunction(h_camera, cameraCallback, this, nullptr);
    }
    init_done = true;
    return true;
}

bool CameraWrapper::read(cv::Mat &src) {
    if(init_done) {
        if (mode == 0)return readProcessed(src);
        if (mode == 1)return readRaw(src);
        if (mode == 2)return readCallback(src);
    } else {
        return false;
    }
}

bool CameraWrapper::readRaw(cv::Mat &src) {
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 500) == CAMERA_STATUS_SUCCESS) {
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }

        iplImage = cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, 1);

        cvSetData(iplImage, pby_buffer, frame_info.iWidth);  //�˴�ֻ������ָ�룬��ͼ������ݿ��������赣��ת��Ч��

        src = cv::cvarrToMat(iplImage).clone();

        //�ڳɹ�����CameraGetImageBuffer�󣬱������CameraReleaseImageBuffer���ͷŻ�õ�buffer��
        //�����ٴε���CameraGetImageBufferʱ�����򽫱�����һֱ������ֱ�������߳��е���CameraReleaseImageBuffer���ͷ���buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);

        return true;
    } else {
        src = cv::Mat();
        return false;
    }
}

bool CameraWrapper::readProcessed(cv::Mat &src) {
//	cerr << "Get-1" << endl;
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 500) == CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(h_camera, pby_buffer, rgb_buffer,
                           &frame_info);  // this function is super slow, better not to use it.
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, channel);
        cvSetData(iplImage, rgb_buffer, frame_info.iWidth * channel);  //�˴�ֻ������ָ�룬��ͼ������ݿ��������赣��ת��Ч��
        src = cv::cvarrToMat(iplImage).clone();
        //�ڳɹ�����CameraGetImageBuffer�󣬱������CameraReleaseImageBuffer���ͷŻ�õ�buffer��
        //�����ٴε���CameraGetImageBufferʱ�����򽫱�����һֱ������ֱ�������߳��е���CameraReleaseImageBuffer���ͷ���buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);
        return true;
    } else {
        src = cv::Mat();
        return false;
    }
}

bool CameraWrapper::readCallback(cv::Mat &src) {
    systime ts, te;
    getsystime(ts);
    while(src_queue.empty()){
        getsystime(te);
        if(getTimeIntervalms(te, ts) > 500){
            return false;
        }
    }
    return src_queue.pop(src);
}

CameraWrapper::~CameraWrapper() {
    CameraUnInit(h_camera);
    //ע�⣬�ȷ���ʼ������free
    if (rgb_buffer != nullptr)
        free(rgb_buffer);
}
