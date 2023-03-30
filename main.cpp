/*
*	@Author: USTL-COD
*	@Date:	 2022.06.05
*	@Brief:  multi-thread starts//奇点，利用互斥锁mutex加锁线程
*/

#include "AngleSolver.h"
#include "FrameReceiver.h"
#include "SerialTest.h"

using namespace cv;
using namespace std;

mutex mtx_pre;
bool rwable_pre=false;
condition_variable cv_pre;
mutex mtx_serialr;
bool rwable_serialr=false;
condition_variable cv_serialr;
mutex mtx_serialt;
bool rwable_serialt=false;
condition_variable cv_serialt;
mutex mtx_date;
bool rwable_date=false;
condition_variable cv_date;

int frameSize=6,stateNum = 5, measureNum = 5, controlNum = 5;
double timecolck=0;//as the same of frame
Mat src = Mat::zeros(480, 640, CV_8UC3);   // Transfering buffer
KalmanFilter KF(stateNum, measureNum, controlNum);//状态值测量值5×1向量(x,y,△x,△y,distance)
FrameReceiver frameStream(frameSize);
ArmorMedia armorReceive;
ArmorMedia armorTransmit;
//import armor detector
ArmorDetector armorDetector;
//import angle solver
AngleSolver angleSolver;
SerialPort myserial;

int main()
{
    XInitThreads();
    thread thread1(imageUpdatingThread);
    thread thread2(preimgUpdatingThread);
    thread thread3(serialRUpdatingThread);
    thread thread4(datasolutingThread);
    thread thread5(serialTUpdatingThread);
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();
    return 0;
}

void imageUpdatingThread(void)
{
    frameStream.getFrames();
}

void preimgUpdatingThread(void)
{
    Frame frame;
    int n=0;
    while(true)
    {
        // FPS
        double t = (double)getTickCount();

        if(frameStream.getLatest(frame))
        {
            src=frame.frame;
        }
        else
        {
            //cout<<"getLast test"<<endl;
            continue;
        }
        //装甲板检测识别子核心集成函数
        armorDetector.prerun(src);

        //FPS
        t = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_preimg:"<<t*1000<<"n_pre:"<<n++<<endl;
        //printf("FPS_preimg: %f\n", 1/t);

        timecolck=frame.ftime;
        cout<<"timecolck:"<<timecolck;

        rwable_serialr=true;
        cv_serialr.notify_one();
    }
}


void serialRUpdatingThread(void)
{

    int n=0;
    while(true)
    {
        // FPS
        double t = (double)getTickCount();

        myserial.SerialWaterFish(armorReceive.enemycolor,armorReceive.targetnum,
                                 armorReceive.predict,armorReceive.bullet_v,armorReceive.gyro_pitch,armorReceive.gyro_yaw);
        //FPS
        t = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_serial:"<<t*1000<<"n_serial:"<<n++<<endl;
        //printf("FPS_serial: %f\n", 1/t);

        rwable_date=true;
        cv_date.notify_one();
    }
}

void datasolutingThread(void)
{
    //waitKey(3000);
    int n=0;
    mutex copy_pre;
    while (true)
    {
        // FPS
        double t = (double)getTickCount();

        std::unique_lock<std::mutex> lck_serialr(mtx_serialr);
        cv_serialr.wait(lck_serialr,[](){return rwable_serialr;});

        std::unique_lock<std::mutex> lck_date(mtx_date);
        cv_date.wait(lck_date,[](){return rwable_date;});

        copy_pre.lock();
        armorDetector.daterun();
        copy_pre.unlock();

        //FPS
        double t1 = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_copy:"<<t1*1000<<endl;

        armorDetector.setEnemyColor(armorReceive.enemycolor); //here set enemy color
        armorDetector.setTargetNum(armorReceive.targetnum);
        angleSolver.setBulletSpeed(armorReceive.bullet_v);
        angleSolver.setGyroYaw(armorReceive.gyro_yaw);
        angleSolver.setGyroPitch(armorReceive.gyro_pitch);

        //给角度解算传目标装甲板值的实例
        if (armorDetector.isFoundArmor())
        {
            armorDetector.getTargetInfo(armorReceive.contourPoints, armorReceive.centerPoint, armorReceive.type);
            angleSolver.getAngle(armorReceive.contourPoints, armorReceive.centerPoint, armorReceive.type,
                armorTransmit.yaw, armorTransmit.pitch, armorTransmit.distance, armorTransmit.tar_predict);
        }

        if (armorDetector.isFoundArmor())
        {
            //printf("Found Target! Center(%d,%d)\n", (int)centerPoint.x, (int)centerPoint.y);
            cout << "Yaw: " << armorTransmit.yaw << "Pitch: " << armorTransmit.pitch << "Distance: " << armorTransmit.distance << endl;
        }

#ifdef DEBUG_MODE
            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出,disable motor action
            //param:
            //		1.showSrcImg_ON,		  是否展示原图
            //		2.bool showSrcBinary_ON,  是否展示二值图
            //		3.bool showLights_ON,	  是否展示灯条图
            //		4.bool showArmors_ON,	  是否展示装甲板图
            //		5.bool textLights_ON,	  是否输出灯条信息
            //		6.bool textArmors_ON,	  是否输出装甲板信息
            //		7.bool textScores_ON	  是否输出打击度信息
            //					    1  2  3  4  5  6  7
        armorDetector.showDebugInfo(0, 0, 1, 1, 0, 0, 0);
        if (armorDetector.isFoundArmor())
        {
            //角度解算调试参数是否输出
            //param:
            //		1.showCurrentResult,	  是否展示当前解算结果
            //		2.bool showTVec,          是否展示目标坐标
            //		3.bool showP4P,           是否展示P4P算法计算结果
            //		4.bool showPinHole,       是否展示PinHole算法计算结果
            //		5.bool showCompensation,  是否输出补偿结果
            //		6.bool showCameraParams	  是否输出相机参数
            //					      1  2  3  4  5  6
            angleSolver.showDebugInfo(0, 0, 0, 0, 0, 0);
        }
        waitKey(1);
#endif

        //FPS
        t = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_date:"<<t*1000<<"n_date:"<<n++<<endl;
        printf("FPS_date: %f\n", 1/t);

        rwable_serialr=false;
        rwable_date=false;
        rwable_serialt=true;
        cv_serialt.notify_one();
    }
}
void serialTUpdatingThread(void)
{
    while(true)
    {
        std::unique_lock<std::mutex> lck_serialt(mtx_serialt);
        cv_serialt.wait(lck_serialt,[](){return rwable_serialt;});

        myserial.SerialFireFish(armorTransmit.yaw, armorTransmit.pitch,armorTransmit.distance, true, armorDetector.isFoundArmor());

        rwable_serialt=false;
    }
}
