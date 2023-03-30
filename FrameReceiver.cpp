#include "FrameReceiver.h"
FrameReceiver::FrameReceiver(size_t size):
    frames(size),
    mutexs(size),
    headIdx(-1),
    //tailIdx(0),
    _lastGetTimeStamp(0.0)
{

}
bool FrameReceiver::fpush(const Frame& frame)
{
    const size_t newHeadIdx = (headIdx + 1) % frames.size();

    //try for 1ms to lock
    unique_lock<timed_mutex> lock(mutexs[newHeadIdx],chrono::milliseconds(1));
    if(!lock.owns_lock())
    {
        cout<<"!lock.owns_loc"<<endl;
        return false;
    }

    frames[newHeadIdx] = frame;
//    if(newHeadIdx == tailIdx)
//    {
//        tailIdx = (tailIdx + 1) % frames.size();
//        //cout<<"tailIdx:"<<tailIdx<<endl;
//    }
    headIdx = newHeadIdx;

    return true;
}
//获取最新一帧图像
bool FrameReceiver::getLatest(Frame& frame)
{
    volatile const size_t _headIdx = headIdx;

    //try for 1ms to lock
    unique_lock<timed_mutex> lock(mutexs[_headIdx],chrono::milliseconds(1));
    if(!lock.owns_lock() ||
       frames[_headIdx].frame.empty() ||
       frames[_headIdx].ftime == _lastGetTimeStamp)
    {
        return false;
    }

    frame = frames[headIdx];
    _lastGetTimeStamp = frames[headIdx].ftime;

    return true;
}
void FrameReceiver::getFrames(void)
{
    int nRet = MV_OK;
    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);

    // select device and create handle   |unsigned int nIndex = 0;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[unsigned(0)]);
    if (MV_OK != nRet)
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);

    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    // Set trigger mode as off
        /*nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }
        */
        // Get payload size 获取数据包大小
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);

    // start grab image 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    auto startTime = chrono::high_resolution_clock::now();

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    do
    {
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char* pData = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue);

        if (NULL == pData)
            printf("Allocate memory failed.\n");

        // get one frame from camera with timeout=1000ms
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue, &stImageInfo, 100);

        if (nRet == MV_OK)
        {
            /*printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n",
                stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);*/
        }
        else
        {
            printf("No data[0x%x]\n", nRet);
            free(pData);
            pData = NULL;
            break;
        }

        double timeStamp = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() - startTime)).count();
        //cout << "Capture period: " << timeStamp << " ms" << endl;
        // 数据去转换
        availableImg = Convert2Mat(&stImageInfo, pData);
        //double time111 = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() - startTime)).count();
        //cout << "Capture period111: " << time111 << " ms" << endl;
        if (!fpush(Frame{ availableImg,timeStamp }))
        {cout<<"what is fuck"<<endl;continue;}

        double time222 = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() - startTime)).count();
        //cout << "Capture period222: " << time222 << " ms" << endl;

        // print result
        if (NULL!= availableImg.data)
        {
//            printf("NULL!= availableImg.data\n");
            free(pData);
            pData = NULL;
        }
        else
        {
            free(pData);
            pData = NULL;
        }
        // Stop grab image
        /*nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }*/

        // Close device
        /*nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }*/

        // Destroy handle
        /*nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }*/
    }while (1);
    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }
}
// convert data stream in Mat format
Mat FrameReceiver::Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    static cv::Mat srcImage, dstImage;
    srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC2, pData);
    dstImage = Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3);
    cvtColor(srcImage, dstImage, COLOR_YUV2BGR_YUY2);//bgr
    if (NULL == srcImage.data)
    {
        printf("convert_src is false");
    }
//    imshow("cvt",dstImage);
//    waitKey(1);
    srcImage.release();
    return dstImage;
}

