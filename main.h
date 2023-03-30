/*
*	@Author: USTL-COD
*	@Date:	 2022.04.04
*	@Brief:  This header file include the common head files and define the common structure as well as the functions ect.
*/
#pragma once

#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <X11/Xlib.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>

#define DEBUG_MODE

using namespace cv;
using namespace ml;
using namespace std;

// extern variables
// no variables
extern struct ArmorMedia armorReceive;
extern struct ArmorMedia armorTransmit;
extern KalmanFilter KF;
extern int stateNum, measureNum, controlNum;
extern double timecolck;

/*
* @brief: imageUpdating thread
*/
void imageUpdatingThread(void);

/*
* @brief: serialUpdatingThread thread
*/
void preimgUpdatingThread(void);

/*
* @brief: serialUpdatingThread thread
*/
void serialRUpdatingThread(void);


/*
* @brief: armorDetecting thread
*/
void datasolutingThread(void);

/*
* @brief: serialUpdatingThread thread
*/
void serialTUpdatingThread(void);

/*
    *@brief: the types of armor BIG SMALL 大装甲板 小装甲板
    */
enum class ArmorType
{
    SMALL_ARMOR = 0,
    BIG_ARMOR = 1,
    ERROR_ARMOR=2
};

/*
* @brief: colors in order B G R 颜色B蓝 G绿 R红
*/
enum class Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

struct ArmorMedia
{
    Point2f centerPoint;
    vector<Point2f> contourPoints;
    ArmorType type;
    double yaw = 0;
    double pitch = 0;
    double distance = 0;
    int enemycolor = 0;
    int targetnum = 1;
    bool predict=false;
    int bullet_v=14.8;
    float gyro_yaw=0;
    float gyro_pitch=0;
    Point2f tar_predict =Point2f(320.0f,240.0f);
};
