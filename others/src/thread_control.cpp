#include"thread_control.h"
#include "fstream"
SerialPort serialport;

ofstream outfile;
WrapperHead *video = nullptr;
static volatile unsigned int consumption_index;
static volatile unsigned int produce_index;
static volatile unsigned int gimbal_data_index;
static volatile unsigned int save_image_index;
SerialPort serial_(SERIAL_PATH,0);
void ThreadControl::ImageProduce(){
#ifdef USE_GALAXY
    cout << "---------------ImageProduce Task On-----------------------" << endl;
    CameraDevice galaxy;
    galaxy.init();
    while(1)
    {
        while(produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
        galaxy.getImage(image_);
        ++produce_index;
        END_THREAD;

    }

#endif
#ifndef USE_GALAXY
    cout << "---------------ImageProduce Task On-----------------------" << endl;

    bool ok =true;
    int from_camera = 1;
    if(from_camera)
    {
        video = new CameraWrapper(ENERGY_CAMERA_EXPOSURE,ENERGY_CAMERA_GAIN,2);
    }
    if (video->init()) {
        LOGM("video_source initialization successfully.");
    } else {
        LOGW("video_source unavailable!");
    }
    while(ok)
    {


            while(produce_index - consumption_index >= BUFFER_SIZE)
                END_THREAD;
            ok = checkReconnect(video->read(image_));
            ++produce_index;
            END_THREAD;




    }
#endif





}
#ifdef GET_DATA_THREAD
void ThreadControl::GetData(){
    cout << "------ STM32 DATA RECEVICE TASK ON !!! -------" << endl;
    serial_receive_data rx_data;

    bool mode = 0;
    bool color = 0;
    int yaw_offset = 0;
    int pit_offset = 0;
    int bulletspeed_=0;
    
    cout << "mode" << mode <<endl;
    while(1)
    {
        while(static_cast<int>(gimbal_data_index - consumption_index) >= BUFFER_SIZE)
            END_THREAD;

        serial_.read_data(&rx_data, mode,bulletspeed_,yaw_offset, pit_offset,color);
        other_param.mode = mode;
        other_param.color = color;
	other_param.bulletspeed = bulletspeed_;
        other_param.buff_offset_x = yaw_offset;
        other_param.buff_offset_y = pit_offset;
	cout << "mode" << other_param.mode << " " << "yaw_offset" << other_param.buff_offset_x << " " << "pit_offset" << other_param.buff_offset_y << " " << "bulletspeed" << other_param.bulletspeed <<"color"<<" " << other_param.color << endl;
//        GimDataPro.ProcessGimbalData(raw_gimbal_yaw, dst_gimbal_yaw);
//        float gimbal_data = dst_gimbal_yaw;
//        //        INFO(dst_gimbal_yaw);
//        other_param.gimbal_data = gimbal_data;

        if((gimbal_data_index%50)==0)
        {
            printf("Id: %d, Mode: %d, x:%d, y:%d\r\n", gimbal_data_index, mode, yaw_offset, pit_offset);
        }
        gimbal_data_index++;
        END_THREAD;
    }
}
#endif
void ThreadControl::ImageProcess()
{
    bool ok = true;
//    int stateNum = 6;
//    int measureNum = 3;
//    KalmanFilter KF(stateNum, measureNum, 0);
//    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
//    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum)
//            << 1, 0, 0,0.3,0,0,//A 状态转移矩阵
//               0, 1, 0,0,0.3,0,
//               0, 0, 1,0,0,0.3,
//               0,0,0,1,0,0,
//               0,0,0,0,1,0,
//               0,0,0,0,0,1);
//
////这里没有设置控制矩阵B，默认为零
//    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
//    cv::setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
//    cv::setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
//    cv::setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
//    cv::setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
	cout << "----------Image Process Task On -----------" << endl;

	BuffDetector buff_detector;
    ArmorAcquire armor_acquire;
    serial_transmit_data tx_data;
#ifdef ARMOR_TRACK_BAR
    namedWindow("ArmorParam");
    createTrackbar("brightness_threshold_blue", "ArmorParam", &armor_acquire.brightness_threshold_blue, 255);
    createTrackbar("brightness_threshold_red", "ArmorParam", &armor_acquire.brightness_threshold_red, 255);
    createTrackbar("color_threshold_blue", "ArmorParam", &armor_acquire.color_threshold_blue, 255);
    createTrackbar("color_threshold_red", "ArmorParam", &armor_acquire.color_threshold_red, 255);
#endif
#ifdef BUFF_TRACK_BAR
    namedWindow("BuffParam",WINDOW_NORMAL);
    ///createTrackbar("buff_gray_th", "BuffParam", &buff_detector.gray_th_, 255);
    createTrackbar("buff_color_th", "BuffParam", &buff_detector.color_th_, 255);
    createTrackbar("buff_offset_x_", "BuffParam", &buff_detector.buff_offset_x_, 200);
    createTrackbar("buff_offset_y_", "BuffParam", &buff_detector.buff_offset_y_, 200);
    createTrackbar("world_offset_x", "BuffParam", &buff_detector.world_offset_x_, 1000);
    //createTrackbar("fire_max_cnt", "BuffParam", &buff_detector.auto_control.fire_task.max_cnt_, 200);
    //createTrackbar("reset_cnt", "BuffParam", &buff_detector.auto_control.reset_task.max_cnt_, 200);
    //createTrackbar("repeat_time", "BuffParam", &buff_detector.auto_control.fire_task.repeat_time, 2000);
    //createTrackbar("fire", "BuffParam", &buff_detector.auto_control.fire_task.fire_flag, 1);
    //createTrackbar("repeat fire", "BuffParam", &buff_detector.auto_control.fire_task.repeat_fire_flag, 1);
#endif
#ifdef DEBUG_VIDEO
    VideoCapture cap;
    int index;
    cap.open(BUFF_VIDEO_PATH);
#endif
    Mat image;
    Object final_target;
    float angle_x = 0.0, angle_y = 0.0, distance = 0.0;
    int command = 0;
    char key = '\0';
    ofstream outFile;
    bool dir = false;
    float add_y = 0.0;
    outFile.open("mydata.txt");

    while(1){


#ifdef OPEN_CAMERA
    while(produce_index - consumption_index <= 0){
        END_THREAD;
    }

        image_.copyTo(image);


#else
        cap.read(image);
        if(key == 'f'){
            dir = !dir;
        }
        if(dir)
            flip(image, image, ROTATE_180);
#endif
        if(other_param.mode == 0){
            ++consumption_index;
            //TIME_START(fps)
            command = armor_acquire.ArmorDetectTask(image,other_param);
            armor_acquire.getAngle(angle_x,angle_y,distance);
            //add_y = 1.668e-06*distance*distance-0.01008*distance+11.22;
            //add_y=(-2.742e-10)*pow(distance,3)+(2.689e-06)*pow(distance,2)+(-0.01038*distance)+9.703;
            add_y = (3.73e-07)*pow(distance,2)-0.004691*distance+5.975;

            //TIME_END(fps)
            if(!command){
              angle_x = 90;
              angle_y = 90;
              add_y = 0;
            } 
        }
        else{
            ++consumption_index;
            if (last_mode == 0)
            {
                buff_detector.readXML();
            }
            //TIME_START(fps)
            command = buff_detector.BuffDetectTask(image, other_param);
            //TIME_END(fps)
            if (command)
            {
                buff_detector.getAngle(angle_x, angle_y);
                
                distance = buff_detector.getDistance();
                
            }
            if(!command)
            {
//            dis=0;
                angle_x=90;
                angle_y=90;
                
            }
        }

        if (last_mode == 1 && other_param.mode == 0)
        {
            buff_detector.writeXML();
        }
        last_mode = other_param.mode;
        limit_angle(angle_x, 90);
        static bool fast_flag = false;
        //cout << "angle_x" << angle_x << endl;
        //cout << "angle_y" << angle_y << endl;
        int x = image.rows/2;
        int y = image.cols/2;
        Point p  (y, x);

        //cout << "p.x:" << p.x << "p.y:" << p.y << endl;
//        int dis = buff_detector.distance_;
//
//        int yaw = round(angle_x * 100);
//        int pitch = round(angle_y * 100);


        circle(image, p, 3, Scalar(0, 255, 0), -1);
        //circle(image, buff_detector.armorp, 3, Scalar(255, 0, 0), -1);
        /*cout << "armorp:" << buff_detector.armorp << endl;
        if (buff_detector.armorp.x < p.x)
        {
            cout << "左";
            if (yaw > 0)
            {
                yaw = -yaw;
            }

        }
        else if (buff_detector.armorp.x >= p.x)
        {
            cout << "右";
            if (yaw < 0)
            {
                yaw = -yaw;
            }

        }
        if (buff_detector.armorp.y < p.y)
        {
            cout << "上" << endl;
            if (pitch > 0)
            {
                pitch = -pitch;
            }

        }
        else if(buff_detector.armorp.y >= p.y){
            cout << "下" << endl;
            if (pitch < 0)
            {
                pitch = -pitch;
            }

        }*/
//        LowFilter.apply2(dis,yaw,pitch);
//        dis=LowFilter.output[0];
//        yaw=LowFilter.output[1];
//        pitch=LowFilter.output[2];
//        Mat prediction = KF.predict();
//        randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
//        //Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));
//
//        measurement.at<float>(0) = (float)dis;
//        measurement.at<float>(1) = (float)yaw;
//        measurement.at<float>(2) = (float)pitch;
//        KF.correct(measurement);

//        int _dis = (int)prediction.at<float>(0);
//        int _yaw = (int)prediction.at<float>(1);
//        int _pitch = (int)prediction.at<float>(2);

        // cout << "原数据：" << "[" << buff_detector.distance_ << "," << angle_x << "," << angle_y << "]" << endl;
        //cout << "串口数据：" << "[" << dis << "," << yaw << "," << pitch << "]" << endl;
        //outfile << "串口数据：" << "[" << dis << "," << yaw << "," << pitch << "]" << endl;
        //outfile << "串口=  " << "[ " << dis << "," << yaw << "," << pitch << " ]" << endl;
//        cout << "串口=  " << "[ " << dis << "," << yaw << "," << pitch << " ]" << endl;
#ifdef GET_DATA_THREAD
        //cout << angle_x << " " << angle_y << endl;
        
        cout << "add_y" << add_y << endl;
        if(other_param.mode == 0){

          tx_data.get_xy_data(int16_t(angle_x*32767/90), int16_t((angle_y+add_y)*32767/90));

        //tx_data.get_xy_data(int16_t((angle_x+armor_acquire.add_x)*32767/90), int16_t(angle_y*32767/90));
          cout << "yaw = " << angle_x << ",pitch = " << angle_y << ",dis = " << distance<< endl;
          cout << "final_yaw = " << int16_t(angle_x*32767/90) << ",final_pitch = " << int16_t((angle_y+add_y)*32737/90) << ",dis = " << distance << endl;
          outFile << "yaw = " << angle_x << ",pitch = " << angle_y << ",dis = " << distance<< endl;
          outFile << "final_yaw = " << int16_t(angle_x*32767/90) << ",final_pitch = " << int16_t((angle_y+add_y)*32737/90) << ",dis = " << distance << endl;
        }
        else if(other_param.mode == 1){
          tx_data.get_xy_data(int16_t(angle_x*32767/90), int16_t(angle_y*32767/90));
          cout << "yaw = " << angle_x << ",pitch = " << angle_y << ",dis = " << distance<< endl;
          cout << "final_yaw = " << int16_t(angle_x*32767/90) << ",final_pitch = " << int16_t(angle_y*32737/90) << ",dis = " << distance << endl;
        }

          serial_.send_data(tx_data);
//cout<<"add_x="<<armor_acquire.add_x<<endl;
        
#endif
//        serialport.xy[1] = dis >> 8;
//        serialport.xy[2] = dis & 0xff;
//        serialport.xy[3] = yaw >> 8;
//        serialport.xy[4] = yaw & 0xff;
//        serialport.xy[5] = pitch >> 8;
//        serialport.xy[6] = pitch & 0xff;
//
//        for (int i = 0; i < 8; i++) {
//            serialport.write_com(serialport.xy[i],sizeof(serialport.xy[i]));
//        }
#ifdef WAITKEY
#ifdef IMAGESHOW
        namedWindow("image",WINDOW_NORMAL);

        imshow("image", image);
#endif
        key = waitKey(WAITKEY);
        if (key == 'q')
            end_thread_flag = true;
        else if (key == 's') {
            waitKey(0);
        }

        /*else if (key == 'g') {
            fast_flag = !fast_flag;
        }*/
         if (key == 'm') {
            other_param.mode = !other_param.mode;
        }

        END_THREAD;
#endif
    }
    outfile.close();
    }

#ifdef SAVE_VIDEO_THREAD
void ThreadControl::ImageWrite()
{
    cout << " ------ IMAGE WRITE TASK ON !!! ------" << endl;
    SaveVideo writer;
    INFO(writer.getState());
    while(writer.getState()){
        while(static_cast<int>(produce_index - save_image_index) <= 0){
            END_THREAD;
        }
        Mat img_tmp;
        image_.copyTo(img_tmp);
        if(img_tmp.rows == 360)
            copyMakeBorder(img_tmp, img_tmp, 0, 120, 0, 0, BORDER_CONSTANT, Scalar::all(0));
        writer.updateImage(img_tmp);
        save_image_index++;
    }
}
#endif




void protectDate(int& a, int &b, int &c, int& d, int& e, int& f)
{
    if(a<=0)
        a = 1;
    if(b<=0)
        b = 1;
    if(c<=0)
        c = 1;
    if(d<=0)
        d = 1;
    if(e<=0)
        e = 1;
    if(f<=0)
        f = 1;
}
void limit_angle(float& a, float max)
{
    if (a > max)
        a = max;
    else if (a < -max)
        a = -max;
}
ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
}
