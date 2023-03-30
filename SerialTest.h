#pragma once
#include <main.h>
    #include <stdio.h>      /*标准输入输出定义*/
    #include <stdlib.h>     /*标准函数库定义*/
    #include <unistd.h>     /*Unix 标准函数定义*/
    #include <fcntl.h>      /*文件控制定义*/
    #include <termios.h>    /*PPSIX 终端控制定义*/
    #include <errno.h>      /*错误号定义*/
    #include <stdint.h>
    #include <string.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
using namespace std;
class SerialPort
{
private:
    int fd;
    int start;
    struct  termios options;
    uint8_t recv_buff[28];
    uint8_t send_buff[12] = {	0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x54 };
    int speed_arr[16] = {	B460800,B38400, B19200, B9600, B4800, B2400, B1200, B300,
                            B460800,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
    int name_arr[16] = {	 460800, 38400,  19200,  9600,  4800,  2400,  1200,  300,
                             460800, 38400,  19200,  9600,  4800,  2400,  1200,  300, };
public:
    SerialPort();
    bool Serial_Open(const char* dev);
    int Serial_Set(int speed, int flow_ctrl, int databits, int stopbits, int parity);
    int Serial_Recv(uint8_t* buffer, int size);
    int Serial_Send(uint8_t* buffer, int size);
    void Serial_Close();
    //串口主函数
    void SerialFireFish(double yaw, double pitch,double distance, bool fire, bool find);
    void SerialWaterFish(int& enemycolor,int& targetnum,bool& predict,int& bullet_v,float& gyro_pitch,float& gyro_yaw);
};

