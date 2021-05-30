#include "serialport.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

SerialPort::SerialPort() {}

SerialPort::SerialPort (const char* filename,int buadrate){
    file_name_ = filename;
    buadrate_=buadrate;
    success_ = false;
    fd = open(file_name_,O_RDWR | O_NOCTTY | O_SYNC);
    last_fd = fd;
    if(fd == -1)
    {
        printf("open_port wait to open %s.\n",file_name_);
        return;
    }
    else if(fd != -1)
    {
        fcntl(fd,F_SETFL,0);
        printf("port is open %s.\n",file_name_);
    }
    struct termios port_settings;
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings,B115200);
        cfsetospeed(&port_settings,B115200);
    }
    else if(buadrate_ == 1){
        cfsetispeed(&port_settings,B921600);
        cfsetospeed(&port_settings,B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port
}

void SerialPort::send_data(const struct serial_transmit_data &data)
{
    if(data.size != write(fd,data.raw_data,data.size))
    {
        cout << "!!! send data failure !!!" << fd << endl;
        restart_serial();
        cout << "restart fd" << fd << endl;
    }
}
bool SerialPort::read_data(const struct serial_receive_data *data, bool &mode,int &bulletspeed,  int &buff_offset_x, int &buff_offset_y,bool &my_car_color)
{

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
    unsigned char read_buffer[8];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */

    bytes_read = read(fd,&read_buffer,7); /* Read the data                   */
//    cout << "bytes_read: " << bytes_read;
//    for (int i = 0; i < bytes_read; i++) {
//        cout << "buf " << i << ": " << read_buffer[i];
//    }
//    cout << endl;
    if(bytes_read == -1 || bytes_read == 0)
    {
//        cout << "can not read!" << endl;
//        NOTICE("can not read!",3);
        restart_serial();
        success_ = false;
        return 0;
    }
    //cout<<"data head"<<read_buffer[0]<<","<<read_buffer[6]<<endl;
//    printf("buffer1 = %d\t\buffer1 = %d\t buffer1 = %d\tbuffer1 = %d\t\n", read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4]);
    if(read_buffer[0] == data -> head && read_buffer[6] == data->end)
    {
        NOTICE("Get stm32 data sucessed!!!", 3)
        mode = bool(read_buffer[1]);
//        printf("buffer1 = %d\r\n", read_buffer[1]);
        //my_car_color = bool(read_buffer[2]);
	
        buff_offset_x = char(read_buffer[2]);
        buff_offset_y = char(read_buffer[3]);
	bulletspeed = char(read_buffer[4]);
        my_car_color = bool(read_buffer[5]);
        //printf("buffer5 = %d\r\n", read_buffer[5]);
//        cout << "x: " << buff_offset_x << " y:" << buff_offset_y << endl;
        //        gimbal_data = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
//        cout << gimbal_data<< endl;
        success_ = true;
        return 1;
    }
//    cout << "count "<< cccc << endl;
    success_ = false;
    return 0;
}
void SerialPort::restart_serial(void)
{
//    cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
//    cout << serial_mode << endl;
    if(fd == -1 && last_fd != -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if(fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open",1);
        printf("port is open %s.\n", file_name_);
        last_fd = fd;
    }else
    {
        last_fd = fd;
        return;
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates
        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

}

void serial_transmit_data::get_xy_data(int16_t x, int16_t y)
{
    size = 6;
    raw_data[0] = head;
    raw_data[size-1] = end;
    raw_data[1] = x >>8 ;
    raw_data[2] = x & 0xff;
    raw_data[3] = y >> 8;
    raw_data[4] = y &0xff;

}
