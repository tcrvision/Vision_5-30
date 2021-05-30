#pragma once
#include<math.h>
#include"solve_angle.h"

using namespace cv;
using namespace std;
///角度解算宏定义文件////
//#define simple_solve_angle_for_armor_accquire
//#define predict2
//#define Solve_angle  //简易角度解算
#define Solve_angle_   //角度结算

//systemparam system;

class lightbar
{
public:
    RotatedRect rect;//灯条相关数据
    bool matched;//灯条匹配状态
    size_t match_index;//匹配对应灯条序号
    float match_factor;//匹配强度

    lightbar() :matched(false) {}
    lightbar(const RotatedRect& r)
    {
        rect.size = r.size;
        rect.angle = r.angle;
        rect.center = r.center;
        matched = false;
    }
};

//装甲板信息
class  armor
{
public:
    lightbar Lightbar[2];//装甲板两灯条
    float angle_error;//装甲板误差角度
    Point2i center;//装甲板中心
    Rect2i Rect;//装甲板ROI矩形
    int average_intensity;//装甲板ROI平均色彩强度

    armor();
    //~armor();
    armor(const lightbar& L1, const lightbar& L2);//装甲板两灯条
    bool suitale_size(void) const;//判断装甲板尺寸
    int get_average_intensity(const Mat& frame);//计算装甲板ROI平均色彩强度
    void max_match(vector<lightbar>& LB, size_t i, size_t j);//灯条匹配
    void draw_rect(Mat& frame, Point2f roi_offset_point)const;//画装甲板
    void draw_spot(Mat& frame, Point2f roi_offser_point)const;//画瞄准点
};

class ArmorAcquire
{
public:

    //int brightness_threshold_v = 120;
    int brightness_threshold_blue = 100;
    int brightness_threshold_red = 72;
    int brightness_threshold_m = 255;
    //int color_threshold_v = 100;
    int color_threshold_blue = 130;
    int color_threshold_red = 130;
    int color_threshold_m = 255;

public:
    ArmorAcquire()
    {
        solve_angle_=SolveAngle(CAMERA_PATH,LONG_X,LONG_Y,LONG_Z,PTZ_TO_BARREL);
        t_start_ = getTickCount();
    }
    ~ArmorAcquire() {};

    //自瞄任务
    int ArmorDetectTask(Mat& frame, OtherParam param);
    void getAngle(float &yaw,float &pitch,float &distance)
    {
        yaw=angle_x;
        pitch=angle_y;
        distance=distance_;
    }

    int final_x;
    int final_y;


    double xx;
    double yy;

    float tvec_y_;
    float pitch_;
    float ballet_speed_;
    float add_x;


    float distan;
    float pitch;
    float yaw;
    float roll;

    double armor_w;
    double armor_h;
    double xxx;
    double yyy;
    int add_yy = 0;
    int add_xx = 0;


private:
    //获取图像ROI区域
    Rect getRoi(const Mat& frame);
    Vec3f rotationMatrixToEulerAngles(Mat& R);
    float getarmorPitch(float dist, float tvec_y, float ballet_speed);
    void add_y(float dist);
    //int add_x(int d1,int d2,float car_v,float car_a);
    double limit_angle(double a1,double a2);
    double linear_velocity(float omega,double R);
    double angular_velocity(float theta,float T);
    float add_angle_x(bool is_right,bool is_left,double lv,double av);
    double l_v;
    float a_v;
    double l_a;
    bool angle_left;
    bool angle_right;
    float add_angle_x_;


private:
    struct direction{
        int dir_x;
        double dir_angle_x;
    }dir1,dir2;
    float angle_v=0;
    float u=0;
    int sub_dir;
    int abs_sub_dir;
    bool makeRectSafe(cv::Rect& rect, cv::Size& size)
    {
        if (rect.x < 0)
        {
            rect.x = 0;
        }
        if (rect.x + rect.width > size.width)
        {
            rect.width = size.width - rect.x;
        }
        if (rect.y < 0)
        {
            rect.y = 0;
        }
        if (rect.y + rect.height > size.height)
        {
            rect.height = size.height - rect.y;
        }
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }

    //装甲板识别函数
    bool AcquireArmor(Mat& frame, Rect roi_rect,int& color);

    //装甲板类型
    bool getTypeArmor(bool is_small);
    void setFilter(int filter_size)
    {
        filter_size_ = filter_size;
    }
    void clear()
    {
        history.clear();
    }

    int color_;

public:
    //卡尔曼滤波 预测参数
    int km_Qp = 1000;
    int km_Qv = 1;
    int km_Rp = 1;
    int km_Rv = 1;
    int km_t = 1;
    int km_pt = 60;
    float last_angle = 0;
    float last_v = 0;
    float last_last_v = 0;


private:
    double t_start_;

private:
    //ROI参数
    Rect last_target;
    int lost_cnt = 0;
    int detect_cnt = 0;

public:
    //调试参数
    int short_offset_x = 100;
    int short_offset_y = 100;

private:
    vector<Point2f>points_2d_;
private:
    list<bool>history;
    //装甲板类型（大装甲板和小装甲板）
    int filter_size_ = 5;
    bool is_small_;
    float distance_;
    float angle_x;
    float angle_y;
    SolveAngle solve_angle_;

};
