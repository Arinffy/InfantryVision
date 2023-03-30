#pragma once
#include "main.h"
#include "ArmorDeep.h"
#include "ArmorBox.h"

enum class DetectorState
{
    LIGHTS_NOT_FOUND = 0,
    LIGHTS_FOUND = 1,
    ARMOR_NOT_FOUND = 2,
    ARMOR_FOUND = 3
};

/*
* @brief: Detector function to detect lights from srcImg,match light to armors,
*		  select target and recognize armorNumber
*		  装甲板识别类，实现装甲板两侧灯条的检测，
*		  装甲板的灯条匹配，装甲板的筛选，装甲板数字识别，选择目标等功能
*/
class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();
    /*
     * @brief: load svm model for client
     * @param: the model file path of svm
     */
    void loadSVM(const char* model_path, Size armorImgSize = Size(40, 40));

    /*
     * @brief: set enemyColor  设置敌方颜色
     */
    void setEnemyColor(int enemyColornum);

    /*
    *@brief: for client, set the target armor number 操作手用，设置目标装甲板数字
    */
    void setTargetNum(const int& targetNum);

    /*
     *@brief: reset the ArmorDetector(delete the priviois lights and armors) to start next frame detection 重设检测器（删除原有的灯条和装甲板s）和装甲板状态，以便进行下一帧的检测
     */
    void resetDetector();

    Mat imgRoirect(Mat& src,int roivelocity);

    /*
     * @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并进行图像预处理
     * @param: const Mat& src     源图像的引用
     */
    void setImg(Mat& src);

    /*
     * @brief: find all the possible lights of armor (get lights) 检测所有可能的灯条
     */
    void findLights();

    /*
    *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
    */
    void eraseErrorRepeatArmor(vector<ArmorBox>& armors);

    /*
    * @brief: match lights into armors (get armors) 将识别到的灯条拟合为装甲板
    */
    void matchArmors();

    /*
     *@brief: set the privious targetArmor as lastArmor and then choose the most valuable armor from current armors as targetArmor (set targetArmor and lastArmor)
     *			将上一帧的目标装甲板作为lastArmor选择本帧图像中所有装甲板里面价值最大的装甲板作为目标装甲板
     */
    void setTargetArmor();

    void prerun(Mat& src);

    /*
     *@brief: an integrative function to run the Detector 集成的装甲板检测识别函数
     */
    void daterun();

    /*
     *@brief: return the Detector status 识别程序是否识别到装甲版
     *@return: FOUND(1) NOT_FOUND(0)
     */
    bool isFoundArmor();

    /*
    *@:brief: accordingt to the armorNum priority to set the armorScore 根据优先级增加装甲板打击度
    */
    void setNumScore(const int& armorNum, const int& targetNum, float& armorScore);

    /*
    *@brief: get the distance of two points(a and b) 获取两点之间的距离
    */
    float getPointsDistance(const Point2f& a, const Point2f& b);

    /*
    *@brief: compare a_armor to b_armor according to their distance to lastArmor(if exit, not a default armor) and their area and armorNum
    *		  比较a_armor装甲板与b_armor装甲板的打击度，判断a_armor是否比b_armor更适合打击（通过装甲板数字是否与目标装甲板数字匹配，装甲板与lastArmor的距离以及装甲板的面积大小判断）
    */
    bool armorCompare(const ArmorBox& a_armor, const ArmorBox& b_armor, const ArmorBox& lastArmor, const int& targetNum);

    /*
     *@brief: show all the informations of this frame detection  显示所有信息
     */
    void showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON, bool textLights_ON, bool textArmors_ON, bool textScores_ON);


    /*
     *@brief: get the vertices and type of target Armor for angle solver 将detector的结果输出
     */
    void getTargetInfo(vector<Point2f>& armorVertices, Point2f& centerPoint, ArmorType& type);

private:
    Mat srcImg;  //source image (current frame acquired from camera) 从相机采集的当前的图像帧
    Mat srcImg_pre;  //source image (current frame acquired from camera) 从相机采集的当前的图像帧
    Mat srcImg_binary; //binary image of srcImg 源图像的二值图
    Mat srcImg_binary_pre; //binary image of srcImg 源图像的二值图
    Color enemyColor;  //the color of enemy 敌方颜色
    int targetNum; //number of client's target armor 操作手设定的目标装甲板数字
    vector<LightBar> lights; //all the lightBars find in roiIng 找到的灯条
    vector<vector<Point>> lightContours_pre;  //candidate contours of lights roiIng中的候选灯条轮廓
    vector<ArmorBox> armors; //all the armors matched from lights 识别到的所有装甲板
    ArmorBox targetArmor; //current target for current frame 当前图像帧对应的目标装甲板
    ArmorBox lastArmor;  //previous target for last frame 上一帧图像的目标装甲板
    ArmorDeep classDeep; //class used to get armorImg and classifier the armorNum 获取装甲板图像及识别装甲板数字的类
    DetectorState state; //the state of detector updating along with the program running 装甲板检测器的状态，随着装甲板进程的执行而不断更新
};



