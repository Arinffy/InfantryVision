#include "SerialTest.h"

SerialPort::SerialPort()
{
    Serial_Open("/dev/ttyUSB0");
    Serial_Set(460800, 0, 8, 1, 'N');
}
bool SerialPort::Serial_Open(const char* dev)
{
    char* _dev = new char[32];//32
    strcpy(_dev, dev);
    fd = open(_dev, O_RDWR | O_NOCTTY);       //O_RDWR 读写方式打开；O_NOCTTY 不允许进程管理串口（不太理解，一般都选上）；O_NDELAY 非阻塞（默认为阻塞，打开后也可以使用fcntl()重新设置）
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return false;
    }
    else
    {
        fcntl(fd, F_SETFL, 0);//FNDELAY
        printf("port is open.%d\n", fd);
    }
    return true;
}
int SerialPort::Serial_Set(int speed, int flow_ctrl, int databits, int stopbits, int parity)
{
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return(false);
    }
    cfmakeraw(&options);
    //设置串口输入波特率和输出波特率
    for (int i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch (flow_ctrl)
    {

    case 0://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return (false);
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return (false);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return (false);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 19; /* 读取字符的最少个数为1 */
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd, TCIOFLUSH);//TCIFLUSH

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return (false);
    }
    return (true);
}

int SerialPort::Serial_Recv(uint8_t* buffer, int size)
{
    tcflush(fd, TCIFLUSH);//TCIFLUSH
    int temp= read(fd, buffer, size);
    if (temp != size)
//        printf("Expect to get %d sizes of data,but got %d sizes.\n",size,temp);
    for(int i=0;i<size;i++)
    {
        if(buffer[i]==0x69)
        {
            start=i;
            break;
        }
    }
    uint8_t sumcheck=  (recv_buff[start+1]+recv_buff[start+2]+recv_buff[start+3]+recv_buff[start+4]+recv_buff[start+5]+recv_buff[start+6]+
                        recv_buff[start+7]+recv_buff[start+8]+recv_buff[start+9]+recv_buff[start+10]+recv_buff[start+11])/11;
    if(recv_buff[start+12]!=sumcheck)
    {
        printf("sum: %x ~",sumcheck);
        printf(" %x %x %x %x %x %x %x %x %x %x %x\n",recv_buff[start+1],recv_buff[start+2],recv_buff[start+3],recv_buff[start+4],\
                recv_buff[start+5],recv_buff[start+6],recv_buff[start+7],recv_buff[start+8],recv_buff[start+9],recv_buff[start+10],recv_buff[start+11]);
        exit(0);
        return -1;
    }
    return temp;
}
int SerialPort::Serial_Send(uint8_t* buffer, int size)
{
    int temp= write(fd, buffer, size);
    //cout<<"temp:"<<temp<<"fd:"<<fd<<endl;
    if (temp != size)
        printf("Write error.\n");
    return temp;
}
void SerialPort::Serial_Close()
{
    close(fd);
}
void SerialPort::SerialFireFish(double yaw, double pitch,double distance, bool fire, bool find)
{
    static int flag;
    yaw>0?(pitch>0?flag=4:flag=1):(pitch>0?flag=3:flag=2);
    *(send_buff + 1) = (char)find;
    *(send_buff + 2) = (char)(int)abs(yaw);
    *(send_buff + 3) = (char)abs((int)(yaw * 100)%100);
    *(send_buff + 4) = (char)(int)abs(pitch);
    *(send_buff + 5) = (char)abs((int)(pitch * 100)%100);
    *(send_buff + 6) = (char)flag;
    *(send_buff + 7) = (char)fire;
    *(send_buff + 8) = (char)(*(send_buff + 1)+*(send_buff + 2)+*(send_buff + 3)+*(send_buff + 4)
                             +*(send_buff + 5)+*(send_buff + 6)+*(send_buff + 7));
    *(send_buff + 9) = (char)(int)abs(distance);
    *(send_buff + 10) = (char)abs((int)(distance * 100)%100);
    //printf("send:%d;yawsend:%d.%d",(int)(char)abs((int)(yaw * 100)%100),send_buff[2], send_buff[3]);
    Serial_Send(send_buff, 12);
}
void SerialPort::SerialWaterFish(int& enemycolor,int& targetnum,bool& predict,int& bullet_v,float& gyro_pitch,float& gyro_yaw)
{
    Serial_Recv(recv_buff, 28);
    //1head 2color 3id 4predict 5bullet_int 6bullet_float
    //7gyropitch_int 8gyropitch_float_1 9gyropitch_float_2
    //10gyroyaw_int 11gyroyaw_1 12gyroyaw_float_2
    //13sum 14end
    enemycolor=(int)recv_buff[start+1];
    targetnum=(int)recv_buff[start+2];
    predict=(bool)recv_buff[start+3];
    bullet_v=(int)recv_buff[start+4]+0.01*(int)recv_buff[start+5];
    gyro_pitch=(((int)recv_buff[start+6]>128)?((int)recv_buff[start+6]-256):(int)recv_buff[start+6])
            +0.01*(((int)recv_buff[start+7]>128)?((int)recv_buff[start+7]-256):(int)recv_buff[start+7])
            +0.0001*(((int)recv_buff[start+8]>128)?((int)recv_buff[start+8]-256):(int)recv_buff[start+8])
            -0.55*0.0663225116f;
    gyro_yaw=(((int)recv_buff[start+9]>128)?((int)recv_buff[start+9]-256):(int)recv_buff[start+9])
            +0.01*(((int)recv_buff[start+10]>128)?((int)recv_buff[start+10]-256):(int)recv_buff[start+10])
            +0.0001*(((int)recv_buff[start+11]>128)?((int)recv_buff[start+11]-256):(int)recv_buff[start+11]);
//    printf("gyro:%d,%d",(int)recv_buff[start+9]*57, (int)recv_buff[start+10]);
//    printf("gyro_pitch:%f,%f",gyro_pitch, gyro_yaw);
}

