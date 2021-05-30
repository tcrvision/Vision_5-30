#include "camera_device.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "base.h"

CameraDevice::CameraDevice() {
    status = GX_STATUS_SUCCESS;
    src.create(1024,1280,CV_8UC3);
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    nFrameNum = 0;
}

CameraDevice::~CameraDevice()
{
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    //�ͷ�ͼ�񻺳���buffer
    free(stFrameData.pImgBuf);

    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}

int CameraDevice::init()
{
    // ��ʼ����
    status = GXInitLib();

    if (status != GX_STATUS_SUCCESS)
    {
        return 0;
    }
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return 0;
    }
    status = GXOpenDevice(&stOpenParam, &hDevice);
    std::cout << status << std::endl;
    if (status == GX_STATUS_SUCCESS)
    {
        int64_t nPayLoadSize = 0;
        //��ȡͼ��buffer��С�����涯̬�����ڴ�
        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);

        if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0)
        {
            //����GXGetImage�Ĵ������

            //���ݻ�ȡ��ͼ��buffer��Сm_nPayLoadSize����buffer
            stFrameData.pImgBuf = malloc((size_t)nPayLoadSize);

            // �����ع�ֵ
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 1500);

            status =GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
            //�� ȡ �� �� �� �� �� Χ
            GX_FLOAT_RANGE gainRange;
            status = GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);
            //�� �� �� С �� �� ֵ
            status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMin);
            //�� �� �� �� �� �� ֵ
            status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMax);
            //�� �� �� �� �� �� �� ��
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);


            //���òɼ�ģʽ�����ɼ�
            //            status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            //            status = GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
            //            status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
//            int64_t nWidth   = 640;
//            int64_t nHeight  = 420;
//            int64_t nOffsetX = 0;
//            int64_t nOffsetY = 0;
//            status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
//            status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
//            status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
//            status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);


            //���Ϳ�ʼ�ɼ�����
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
            return 1;
        }
    }
    return 0;
}

void CameraDevice::getImage(Mat &img)
{
    GXFlushQueue(hDevice);
    GXGetImage(hDevice, &stFrameData, 100);
    //usleep(1);

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        //ͼ���ȡ�ɹ�
        char* m_rgb_image=nullptr; //���ӵ�����
        m_rgb_image=new char[stFrameData.nWidth*stFrameData.nHeight*3];
        DxRaw8toRGB24(stFrameData.pImgBuf,m_rgb_image,stFrameData.nWidth, stFrameData.nHeight,RAW2RGB_NEIGHBOUR3,DX_PIXEL_COLOR_FILTER(BAYERBG),false);

        memcpy(src.data,m_rgb_image,stFrameData.nWidth*stFrameData.nHeight*3);

        src.copyTo(img);
//        img = src;
        nFrameNum++;

        //��ͼ����д���...
        delete []m_rgb_image;
        //                        }
    }
}

uint64_t CameraDevice::getFrameNumber()
{
    return nFrameNum;
}
