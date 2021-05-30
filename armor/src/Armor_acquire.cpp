#include"Armor_acquire.h"
armor::armor() {}

armor::armor(const lightbar& L1, const lightbar& L2) {
    Lightbar[0] = L1;
    Lightbar[1] = L2;
    angle_error = fabs(L1.rect.angle - L2.rect.angle);

    Rect.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    Rect.height = static_cast<int>((L1.rect.size.height + L1.rect.size.height) / 2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
    Rect.x = center.x - Rect.width / 3;
    Rect.y = center.y - Rect.height / 3;
    Rect.width *= 2.0 / 3;
    Rect.height *= 2.0 / 3;
}

bool armor::suitale_size(void) const {
    // 两个灯条体型相似 
        //cout<<"Lightbar[0].rect.size.height"<<Lightbar[0].rect.size.height<<endl;
        //cout<<"Lightbar[1].rect.size.height"<<Lightbar[1].rect.size.height<<endl;
    if (Lightbar[0].rect.size.height * 0.4f < Lightbar[1].rect.size.height
        && Lightbar[0].rect.size.height * 1.6f > Lightbar[1].rect.size.height)
    {  
        //cout << "1" << endl;
        float armor_width = fabs(Lightbar[0].rect.center.x - Lightbar[1].rect.center.x);
        if (armor_width > Lightbar[0].rect.size.width
            && armor_width > Lightbar[1].rect.size.width
            && armor_width > (Lightbar[0].rect.size.width + Lightbar[1].rect.size.width) * 2)
          
        {
           // cout << "2" << endl;
            float h_max = (Lightbar[0].rect.size.height + Lightbar[1].rect.size.height) / 2.0f;
            // 两个灯条高度差不大
            if (fabs(Lightbar[0].rect.center.y - Lightbar[1].rect.center.y) < 1.2f * h_max)
            {
                //cout << "3" << endl;
                // 判断长宽比
                if (h_max * 4.0f > Rect.width && h_max < 1.2f * Rect.width)
                {
                    //cout << "4" << endl;
                    //cout << "h_max = " << h_max << endl;
                    //cout << "rect.width = " << Rect.width << endl;
                    return true;
                }
            }
        }
    }
    return false;
}

int armor::get_average_intensity(const Mat& img) {
    if (Rect.width < 1 || Rect.height < 1 || Rect.x < 1 || Rect.y < 1
        || Rect.width + Rect.x > img.cols || Rect.height + Rect.y > img.rows)
        return 255;
    Mat roi = img(Range(Rect.y, Rect.y + Rect.height), Range(Rect.x, Rect.x + Rect.width));
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

void armor::max_match(vector<lightbar>& LB, size_t i, size_t j) {
    RotatedRect R, L;
    if (Lightbar[0].rect.center.x > Lightbar[1].rect.center.x)
    {
        R = Lightbar[0].rect;
        L = Lightbar[1].rect;
    }
    else
    {
        R = Lightbar[1].rect;
        L = Lightbar[0].rect;
    }

    float angle_8 = L.angle - R.angle;
    if (angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = angle_error + 0.5 * angle_8;
    if (!LB.at(i).matched && !LB.at(j).matched)
    {

        LB.at(i).matched = true;
        LB.at(i).match_index = j;
        LB.at(j).matched = true;
        LB.at(j).match_index = i;
        LB.at(i).match_factor = f;
        LB.at(j).match_factor = f;
    }
    if (LB.at(i).matched && !LB.at(j).matched)
    {
        if (f < LB.at(i).match_factor)
        {
            LB.at(LB.at(i).match_index).matched = false;
            LB.at(i).match_factor = f;
            LB.at(i).match_index = j;
            LB.at(j).matched = true;
            LB.at(j).match_factor = f;
            LB.at(j).match_index = i;

        }
    }
    if (LB.at(j).matched && !LB.at(i).matched)
    {
        if (f < LB.at(j).match_factor)
        {
            LB.at(LB.at(j).match_index).matched = false;
            LB.at(j).match_factor = f;
            LB.at(j).match_index = i;
            LB.at(i).matched = true;
            LB.at(i).match_factor = f;
            LB.at(i).match_index = j;
        }
    }
    if (LB.at(j).matched && LB.at(i).matched
        && LB.at(i).match_factor > f && LB.at(j).match_factor > f)
    {
        LB.at(LB.at(j).match_index).matched = false;
        LB.at(LB.at(i).match_index).matched = false;
        LB.at(i).matched = true;
        LB.at(i).match_factor = f;
        LB.at(i).match_index = j;
        LB.at(j).matched = true;
        LB.at(j).match_factor = f;
        LB.at(j).match_index = i;
    }
}

void armor::draw_rect(Mat& img, Point2f roi_offset_point) const
{
    //rectangle(img, Rect + Point_<int>(roi_offset_point), Scalar(0, 0, 255), 2);
}

void armor::draw_spot(Mat& img, Point2f roi_offset_point) const//装甲板中心点
{
    //circle(img, center + Point_<int>(roi_offset_point), 5, Scalar(0, 0, 255), -1);
    //circle(img, center + Point_<int>(roi_offset_point), int(Rect.height / 5), Scalar(0, 0, 255), -1);
}

Rect ArmorAcquire::getRoi(const Mat& frame) {
    Size img_size = frame.size();
    Rect rect_tmp = last_target;
    Rect rect_roi;
    if (rect_tmp.x == 0 || rect_tmp.y == 0
        || rect_tmp.width == 0 || rect_tmp.height == 0
        || lost_cnt >= 15 || detect_cnt % 100 == 0
            )
    {
        last_target = Rect(0, 0, img_size.width, img_size.height);
        rect_roi = Rect(0, 0, img_size.width, img_size.height);
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt < 30)
            scale = 3;
        else if (lost_cnt <= 60)
            scale = 4;
        else if (lost_cnt <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5f);

        rect_roi = Rect(x, y, w, h);

        if (makeRectSafe(rect_roi, img_size) == false)
        {
            rect_roi = Rect(0, 0, img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

bool ArmorAcquire::AcquireArmor(Mat& frame, Rect roi_rect, int& color)
{
    color_ = color;
    Mat roi_image = frame(roi_rect);
    Point2f offset_roi_point(roi_rect.x, roi_rect.y);
    vector<lightbar>LightBar_v;//声明所有可能的灯条容器

#ifdef Auto
    //滑块 方便调整阈值
    /*if (color_ == 0)
    {
        createTrackbar("brightness_red","img",&brightness_threshold_red,brightness_threshold_m);
        createTrackbar("color", "img", &color_threshold_red, color_threshold_m);
    }
    else if (color_ ==1)
    {
        createTrackbar("brightness_blue","img",&brightness_threshold_blue,brightness_threshold_m);
        createTrackbar("color", "img", &color_threshold_blue, color_threshold_m);
    }*/
#endif //Auto


    //灰度处理
    Mat binary_brightness_img, binary_color_img, gray;
    Mat binary_brightness_img1, binary_color_img2;

    cvtColor(roi_image, gray, COLOR_BGR2GRAY);
    //imshow("gray",gray);

    //红蓝通道相减
    vector<cv::Mat> bgr;
    split(roi_image, bgr);
    Mat result_img;
    if (color_ == 0)
    {
        subtract(bgr[2], bgr[0], result_img);
    }
    else if (color_ ==1)
    {
        subtract(bgr[0], bgr[2], result_img);
    }
    //imshow("result_img",result_img);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(1, 1), Point(-1, -1));
    Mat kernel2 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));


    if (color_ == 0)
    {
        threshold(gray, binary_brightness_img, brightness_threshold_red, brightness_threshold_m, THRESH_BINARY);// 二值化
        threshold(result_img, binary_color_img, color_threshold_red, color_threshold_m, THRESH_BINARY);// 二值化
    }
    else if (color_ ==1)
    {
        threshold(gray, binary_brightness_img, brightness_threshold_blue, brightness_threshold_m, THRESH_BINARY);// 二值化
        threshold(result_img, binary_color_img, color_threshold_blue, color_threshold_m, THRESH_BINARY);// 二值化
    }


    //morphologyEx(binary_color_img, binary_color_img,MORPH_CLOSE , kernel);
    morphologyEx(binary_color_img, binary_color_img,MORPH_OPEN , kernel2);
    morphologyEx(binary_brightness_img, binary_brightness_img,MORPH_OPEN , kernel2);
    cv::GaussianBlur(binary_brightness_img, binary_brightness_img, Size(3, 3), 5, 5);
    //cv::GaussianBlur(binary_brightness_img, binary_brightness_img, Size(3, 3), 3, 3);
    cv::GaussianBlur(binary_color_img, binary_color_img, Size(3, 3), 5, 5);
    //cv::dilate(binary_color_img, binary_color_img, Mat());
    cv::dilate(binary_brightness_img, binary_brightness_img, kernel);


    //threshold(gray, binary_brightness_img, 120, 255, THRESH_BINARY);// 二值化
    //threshold(result_img, binary_color_img, 100, 255, THRESH_BINARY);// 二值化

#ifdef Auto
    namedWindow("binary_brightness_img",CV_WINDOW_NORMAL);
    namedWindow("binary_color_img",CV_WINDOW_NORMAL);

    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif //Auto

    // 提取可能的灯条
    vector<vector<Point>> contours_light;// 提取轮廓的数组
    vector<vector<Point>> contours_brightness;// 提取轮廓的数组
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);//最外围轮廓提取
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);//最外围轮廓提取
    //findContours(binary_brightness_img, contours_brightness, RETR_LIST, CHAIN_APPROX_NONE);//最外围轮廓提取

    for (size_t i = 0; i < contours_brightness.size(); i++)
    {
        vector<Point>Points;
        double area = contourArea(contours_brightness[i]);//计算面积
        cout<<"area = "<<area<<endl;
        //cout<<"2e3= "<<2e3<<endl;
        if (area < 20.0 || 3500 < area) continue;
        double high;
        for (size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if (pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0)
            {
                //检测点是否在轮廓内
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                //cout<<"length= "<<length<<endl;
                if (length > 15 && length < 4000)
                {
                    Points = contours_brightness[i];
                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
                    //RotatedRect RRect = fitEllipse(Points);
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    //Point2f* vertices = new cv::Point2f[4];
                    //RRect.points(vertices);

                    //画灯条
                    for (int i = 0; i < 4; i++)
                    {
                        line(frame, rect_point[i] + offset_roi_point, rect_point[(i + 1) % 4] + offset_roi_point, Scalar(0, 255, 255), 2, 8);
                    }

                    high = RRect.size.height;

                    if (RRect.angle > 90.0f) {
                        RRect.angle = RRect.angle - 180.0f;
                    }


                    // 灯条信息
                    //putText(frame, to_string(RRect.angle), RRect.center + Point2f(2, 2) + offset_roi_point,
                    //FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

                    //imshow("img", img);
                    if (fabs(RRect.angle) <= 70)
                    {
                        //cout<<"angle = "<<RRect.angle<<endl;
                        // 超过一定角度的灯条不要
                        lightbar r(RRect);
                        LightBar_v.push_back(r);
                    }
                }

                /* cout << "第" << i+1 << "个面积为" << area << endl;
                cout << "第" << i+1 << "个周长为" << length << endl;*/
                break;
            }
        }
    }
    //寻找可能的装甲板
    for (size_t i = 0; i < LightBar_v.size(); i++)
    {
        for (size_t j = i + 1; j < LightBar_v.size(); j++) {
            armor arm_tmp(LightBar_v.at(i), LightBar_v.at(j));
            if (arm_tmp.angle_error < 8.0f) {
                // 装甲板信息
                //putText(frame, to_string(arm_tmp.Rect.width / (arm_tmp.Rect.height + 0.0001)),
                //arm_tmp.center + Point_<int>(offset_roi_point), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                if (arm_tmp.suitale_size()) {
                    if (arm_tmp.get_average_intensity(gray) < 50)
                    {
                        arm_tmp.max_match(LightBar_v, i, j);
                    }
                }
            }
        }
    }

    // 分类装甲板
    vector<armor> final_armor_list;
    for (size_t i = 0; i < LightBar_v.size(); i++)
    {
        if (LightBar_v.at(i).matched)
        {
            LightBar_v.at(LightBar_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp(LightBar_v.at(i), LightBar_v.at(LightBar_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // 选择装甲板
    float dist = 1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width / 2, roi_rect.height / 2);
    float dx, dy;

    for (size_t i = 0; i < final_armor_list.size(); i++) {
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
        if (dx + dy < dist) {
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
        final_armor_list.at(i).draw_rect(frame, offset_roi_point);
        found_flag = true;
    }

    //rectangle(frame, roi_rect, Scalar(255, 255, 255), 1);
    //imshow("img", frame);

    RotatedRect target_rect;
    if (found_flag)
    {
        target.draw_spot(frame, offset_roi_point);
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if (target.Lightbar[0].rect.center.x > target.Lightbar[1].rect.center.x)
        {
            R = target.Lightbar[0].rect;
            L = target.Lightbar[1].rect;
        }
        else
        {
            R = target.Lightbar[1].rect;
            L = target.Lightbar[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];

        // 计算补偿，用于调试调整准心
        Point2f offset_point;
        offset_point = Point2f(0, 0) - Point2f(-add_xx, -add_yy);

        points_2d_.clear();
        vector<Point2f> points_roi_tmp;
        for (int i = 0; i < 4; i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point + offset_point);
            //putText(frame, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);//装甲板4个点
            //circle(frame, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
        }
        // 计算当前装甲板类型
        float armor_h = target.Rect.height;
        float armor_w = target.Rect.width;
        if (armor_w / armor_h < 2.3f)
            is_small_ = 1; //small armor
        else
            is_small_ = 0; //big armor

        //计算ROI的相关参数
        last_target = boundingRect(points_roi_tmp);

#ifdef Auto
        //画装甲板
        rectangle(frame, last_target, Scalar(255, 0, 0), 2);//灯条拟合装甲板
        rectangle(frame, Point(points_2d_.at(0).x, points_2d_.at(0).y - 30),
                  Point(points_2d_.at(2).x, points_2d_.at(2).y + 30), Scalar(255, 0, 0), 2);//计算装甲板

        if (color_ == 1)
        {
            //rectangle(frame, Point(points_2d_.at(0).x - 130, points_2d_.at(0).y - 120),
            //Point(points_2d_.at(2).x + 130, points_2d_.at(2).y + 120), Scalar(255, 0, 0), 2);//计算车
            //rectangle(frame, Point(points_2d_.at(0).x - (0.1*points_2d_.at(0).x), points_2d_.at(0).y - (0.3*points_2d_.at(0).y)),
            //Point(points_2d_.at(2).x + (0.1 * points_2d_.at(2).x), points_2d_.at(2).y + (0.3 * points_2d_.at(2).y)), Scalar(255, 0, 0), 2);
            //putText(frame, "car", Point(points_2d_.at(0).x - 100, points_2d_.at(0).y - 120), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 1);

        }
        else
        {
            //rectangle(frame, Point(points_2d_.at(0).x - 100, points_2d_.at(0).y - 150),
            //Point(points_2d_.at(2).x + 100, points_2d_.at(2).y + 150), Scalar(0, 0, 255), 1);//计算车
            //putText(frame, "car_color:red", Point(points_2d_.at(0).x - 100, points_2d_.at(0).y - 120), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 1);

        }
#endif //Auto

        lost_cnt = 0;
    }
    else
    {
        //计算ROI的相关参数
        lost_cnt++;
    }
    detect_cnt++;

    xx = target.center.x + offset_roi_point.x;
    yy = target.center.y + offset_roi_point.y;

    final_x = target.center.x + offset_roi_point.x;
    final_y = target.center.y + offset_roi_point.y;

    //imshow("img", frame);
    return found_flag;
}

bool ArmorAcquire::getTypeArmor(bool is_small)
{
    if (history.size() < filter_size_) {
        history.push_back(is_small);
    }
    else
    {
        history.push_back(is_small);
        history.pop_front();
    }

    int vote_cnt[2] = { 0 };
    for (std::list<bool>::const_iterator it = history.begin(); it != history.end(); ++it) {
        *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
    }

    if (vote_cnt[0] == vote_cnt[1])
        return is_small;
    return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
}

bool isRotationMatrix(Mat& R)
{
    Mat RT;
    Rodrigues(R, RT);
    //cout << "旋转矩阵" << dst << endl;
    Mat shouldeBeIdentity = RT * R;
    Mat II = Mat::eye(3, 3, shouldeBeIdentity.type());
    return norm(II, shouldeBeIdentity) < 1e-6;
}

Vec3f ArmorAcquire::rotationMatrixToEulerAngles(Mat& R)
{
    //assert(isRotationMatrix(R));
#define PI 3.14159

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    float theta_x, theta_y, theta_z;

    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    pitch = x * (180 / PI);
    yaw = y * (180 / PI);
    roll = z * (180 / PI);


    //cout << "pitch = " << pitch << endl;
    //cout << "yaw = " << yaw << endl;
    //cout << "roll = " << roll << endl;

    return Vec3f(x, y, z);

}

float ArmorAcquire :: getarmorPitch(float dist, float tvec_y, float ballet_speed)
{
    float y_temp, y_actual, dy;
    float a = 0.0;
    float GRAVITY = 9.7887f;
    y_temp = tvec_y;
    for (int i = 0; i < 10; i++)
    {
        a = (float)atan2(y_temp, dist);
        float t, y = 0.0;
        t = dist / (ballet_speed * cos(a));
        y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.01)
        {
            break;
        }
    }
    return a;
}

int ArmorAcquire::ArmorDetectTask(Mat& frame, OtherParam param) {
    //获取外部数据;
    color_ = param.color;

    //获得历史ROI
#ifdef ROI_ENABLE
    Rect roi = getRoi(frame);
#else
    Size img_size = frame.size();
    Rect roi = Rect(0, 0, img_size.width, img_size.height);
#endif

    if (AcquireArmor(frame, roi, param.color)) {

#ifdef Solve_angle_
        bool final_armor_type=getTypeArmor(is_small_);
        solve_angle_.Generate3DPoints((uint)final_armor_type,Point2f());
        solve_angle_.getAngle(points_2d_,param.bulletspeed,angle_x,angle_y,distance_);
        //add_y(distance_);
#endif //Solve_angle_
#ifdef Predict
        ///间隔时间计算///
        double t_tmp=getTickCount();
        double delta_t=(t_tmp-t_start_)/getTickFrequency();
        t_start_=t_tmp;
        cout<<"delta_t = "<<delta_t<<endl;
        float gim_and_pnp_angle_x=-param.gimbal_data+angle_x;
        ///v///
        angle_v=(gim_and_pnp_angle_x - last_angle)/delta_t;
        last_angle = gim_and_pnp_angle_x;
        ///a///
        u =(angle_v-last_v)/delta_t;
        last_v=angle_v;
        //cout<<"v = "<<angle_v<<",a = "<<u<<endl;

        //dir1.dir_x=final_x;
        //add_x(dir1.dir_x,dir2.dir_x,angle_v,u);
        //dir2.dir_x=dir1.dir_x;

        dir1.dir_angle_x = angle_x;
        limit_angle(dir1.dir_angle_x,dir2.dir_angle_x);
        dir2.dir_angle_x=dir1.dir_angle_x;

        angular_velocity(l_a,delta_t);
        linear_velocity(a_v,distance_);
#endif
	//add_angle_x(angle_right,angle_left,l_v,l_a);
        //cout << "param.add_x=" << add_x << endl;
        return 1;
    }
    final_x = 0;
    final_y = 0;
    distan = 0;
    pitch = 0;
    pitch_ = 0;
    yaw = 0;
    angle_y=90;
    angle_x=90;
    distance_=0;
    angle_v=0;
    u=0;
    add_xx=0;
    add_yy=0;
    //add_x = 0;
    return 0;

}
void ArmorAcquire::add_y(float dist) {
    if(dist>1000&&dist<4000)
    {
        add_yy = 30;
    }
    else if(dist>4000 && dist < 6000)
    {
        add_yy = 45;
    }
    else if(dist < 1000)
    {
        add_yy = 10;
    }
    else if(dist >6000)
    {
        add_yy = 50;
    }

}


double ArmorAcquire::angular_velocity(float theta, float T) {
    a_v=round(l_a/T);
    cout<<"a_v= "<<a_v<<endl;
    return a_v;
}
double ArmorAcquire::linear_velocity(float omega, double R) {
    l_v=round(omega*(R/1000));
    cout<<"l_v= "<<l_v<<endl;
    return l_v;

}
double ArmorAcquire::limit_angle(double a1, double a2) {
    double theta_ag;
    theta_ag = a1-a2;
    if(theta_ag<-0.01)
    {
        angle_left =1;
        angle_right = 0;
        //cout<<"angle left"<<endl;
    }
    else if(theta_ag > 0.03)
    {
        angle_right = 1;
        angle_left =0;
        //cout<<"angle right"<<endl;
    }
    else if(theta_ag <= 0.03&&theta_ag>=0)
    {
        angle_left =0;
        angle_right = 0;
        //cout<<"angle stop"<<endl;
    }
    l_a = abs(a1-a2);
    //cout<<"l_a = "<<round(l_a)<<endl;
    //cout<<"theta_ag"<<theta_ag<<endl;
    return l_a;
}
float ArmorAcquire::add_angle_x(bool is_right, bool is_left, double lv, double av) {
 
    //if(is_right == 0&& is_left == 0)
      //  other_param.add_x = 0;
    if(is_left == 1 && is_right ==0)
    {
        add_x= -(lv+av)/10;
    }
    else if(is_right == 1 && is_left ==0)
    {
        add_x=(lv+av)/10;
    }
    else 
    {
        add_x =0;
    }
	cout<<"add_angle_x="<<add_x<<endl;
        return 1;
}
