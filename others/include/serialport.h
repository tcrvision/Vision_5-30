#pragma once
#include <iostream>
#include "base.h"
#include<cstdio>      /*标准输入输出定义*/
#include<cstdlib>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>  /*数据类型，比如一些XXX_t的那种*/
#include<sys/stat.h>   /*定义了一些返回值的结构，没看明白*/
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<cerrno>      /*错误号定义*/

#define serialport_xy
//#define serialport_py


using namespace std;
struct serial_transmit_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0x80;
    unsigned char end = 0x7f;
    void get_xy_data(int16_t x,int16_t y);
};
struct serial_receive_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0x80;
    unsigned char end = 0x7f;

};
class SerialPort{
public:
    SerialPort();
    SerialPort(const char* filename, int buadrate);
    void restart_serial(void);
    void send_data(const struct serial_transmit_data &data);
    bool read_data(const struct serial_receive_data *data, bool &mode,int &bulletspeed,int &buff_offset_x, int &buff_offset_y,bool &my_car_color);
    int fd;
    int last_fd;
    bool success_;
private:

    const char* file_name_;
    int buadrate_;
    float last_bullet_speed;
};

