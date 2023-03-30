#pragma once
#include "MvCameraControl.h"
#include "./main.h"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

struct Frame
{
    Mat frame;
    //size_t fsum;
    double ftime;
};
class FrameReceiver
{
public:
    FrameReceiver(size_t size);
    ~FrameReceiver()=default;
    bool fpush(const Frame& frame);
    bool getLatest(Frame& frame);
    void getFrames(void);
    Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);
private:
    vector<Frame> frames;
    vector<timed_mutex> mutexs;
    size_t headIdx=0;
    size_t tailIdx=0;
    double _lastGetTimeStamp=0.0;
    Mat availableImg;
};


