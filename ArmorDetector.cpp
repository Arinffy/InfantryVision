#include "ArmorDetector.h"
ArmorDetector::ArmorDetector()
{
    //Set armor detector prop
    loadSVM("/home/cod-1th/Desktop/file_SVM/Infantry1_0123467_0623.xml");
    state = DetectorState::LIGHTS_NOT_FOUND;
}

ArmorDetector::~ArmorDetector() {}

/*
 *@brief: load SVM model 载入svm模型
 *@param: the path of svm xml file and the size of training images svm模型路径及训练集的图像大学
 */
void ArmorDetector::loadSVM(const char* model_path, Size armorImgSize)
{
    classDeep.loadSvmModel(model_path, armorImgSize);
}


/*
* @brief: set enemyColor  设置敌方颜色
*/
void ArmorDetector::setEnemyColor(int enemyColornum) {
    enemyColornum == 0 ? this->enemyColor = Color::BLUE : this->enemyColor = Color::RED;
}

/*
* @brief: set enemyNum  设置目标装甲板数字
*/
void ArmorDetector::setTargetNum(const int& targetNum)
{
    this->targetNum = targetNum;
}

void ArmorDetector::resetDetector()
{
    state = DetectorState::LIGHTS_NOT_FOUND;
    lights.clear();
    armors.clear();
}

Mat ArmorDetector::imgRoirect(Mat& src,iprint nt roivelocity)
{
//    static int roilog=0;
//    if(state==DetectorState::ARMOR_FOUND)
//    {
//        if(((targetArmor.center.x>307&&targetArmor.center.x<717)&&
//            (targetArmor.center.y>240&&targetArmor.center.y<560))||roilog==roivelocity)
//        {
//            roilog++;
//            if(roilog>roivelocity)
//            {
//                Rect rect(256,200,512,400);
//                Mat roi=src(rect);
//                roilog=roivelocity;
//                return roi;
//            }
//        }
//        return src;
//    }
//    else
//    {
//        return src;
//    }
}

/*
* @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并设置ROI区域（当ROI模式开启，且上一帧找到目标装甲板时）
* @param: const Mat& src     源图像的引用
*/
void ArmorDetector::setImg(Mat& src) {
    //imgRoirect(src,30).copyTo(srcImg);
    //FPS
     double t = (double)getTickCount();
    //src.copyTo(srcImg);  //deep copy src to srcImg 深（值）拷贝给srcImg
    srcImg_pre=src;

    t = (getTickCount() - t) / getTickFrequency();
    //cout<<"wastetime222:"<<t*1000<<endl;
     if(srcImg_pre.empty()){cout<<"picture is eeeor"<<endl;}
    classDeep.load_pre(srcImg_pre); //srcImg for classifier, warp perspective  载入classifier类成员的srcImg，用于透射变换剪切出装甲板图
    srcImg_binary_pre = Mat::zeros(srcImg_pre.size(), CV_8UC1); //color feature image
    //pointer visits all the data of srcImg, the same to bgr channel split 通道相减法的自定义形式，利用指针访问，免去了split、substract和thresh操作，加速了1.7倍
    //data of Mat  bgr bgr bgr bgr
    uchar* pdata = (uchar*)srcImg_pre.data;
    uchar* qdata = (uchar*)srcImg_binary_pre.data;
    int srcData = srcImg_pre.rows * srcImg_pre.cols;

    if (srcImg_pre.isContinuous())
    {
        if (enemyColor == Color::RED)
        {
            for (int i = 0; i < srcData; i++)
            {
                if (*(pdata + 2) - *pdata > armorLights.color_threshold)
                    *qdata = 255;
                pdata += 3;
                qdata++;
            }
        }
        else if (enemyColor == Color::BLUE)
        {
            for (int i = 0; i < srcData; i++)
            {
                if (*pdata - *(pdata + 2) > armorLights.color_threshold)
                    *qdata = 255;
                pdata += 3;
                qdata++;
            }
        }
    }
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5)); //kernel for dilate;  shape:ellipse size:Size(3,3) 膨胀操作使用的掩膜
    dilate(srcImg_binary_pre, srcImg_binary_pre, kernel); //dilate the roiImg_binary which can make the lightBar area more smooth 对roiIng_binary进行膨胀操作，试得灯条区域更加平滑有衔接
    kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    erode(srcImg_binary_pre, srcImg_binary_pre, kernel);
    //
    Mat contourImg; //image for the useage of findContours avoiding the unexpected change of itself 给findContours用的图像，防止findContours改变roiImg
    srcImg_binary_pre.copyTo(contourImg); //a copy of roiImg, contourImg
    //最耗时的操作，优化方向
    findContours(contourImg, lightContours_pre, 0, 2); //CV_RETR_EXTERNAL = 0, CV_CHAIN_APPROX_SIMPLE = 2
}

/*
* @brief: find all the possible lights of armor  检测所有可能的灯条
*/
void ArmorDetector::findLights() {
    vector<vector<Point>> lightContours = lightContours_pre;  //candidate contours of lights roiIng中的候选灯条轮廓

    RotatedRect lightRect;  //RotatedRect for fitEllipse 拟合椭圆来的灯条旋转矩形
    RotatedRect miliRect;
    LightBar light;  //template light 临时灯条
    for (const auto& lightContour : lightContours) {//对每个轮廓都进行处理

        //筛选掉噪声
        if (lightContour.size() <= 6 || contourArea(lightContour) < armorLights.min_area) continue;

        lightRect = fitEllipse(lightContour); //lightContour fits into a RotatedRect 拟合椭圆
        miliRect = minAreaRect(lightContour);
        light = LightBar(lightRect,miliRect);//construct to a lightBar 构造为灯条

        if (abs(light.angle) > armorLights.max_angle) continue; //angle filter 角度筛选，滤去一些竖直偏角偏大的

        lights.emplace_back(light);
    }
    if (lights.size() < 2) {
        state = DetectorState::LIGHTS_NOT_FOUND; //if lights is less than 2, then set state not found lights 灯条少于两条则设置状态为没找到灯条
        return; //exit
    }

    // sort the lightBars from left to right 将灯条从左到右排序
    sort(lights.begin(), lights.end(),
        [](LightBar& a1, LightBar& a2) {
            return a1.center.x < a2.center.x; });
    state = DetectorState::LIGHTS_FOUND;
    return;
}

/*
 *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
 */
void ArmorDetector::eraseErrorRepeatArmor(vector<ArmorBox>& armors)
{
    size_t length = armors.size();
    vector<size_t> errorlist(length);
    for (size_t i = 0; i < length; i++){
        //判断倒影装甲板||debug状态***********************************//
        //(armors[i].center.y > 380) ? errorlist[i] = 1 : errorlist[i] = 0;
        for (size_t j = i + 1; j < length; j++)
        {
            //检测游离装甲板
            //左判断距离，单端优先相信短距离
            if (armors[i].l_index == armors[j].l_index)
                (getPointsDistance(armors[i].l_light.center, armors[i].r_light.center) <
                    getPointsDistance(armors[j].l_light.center, armors[j].r_light.center)) ?
                errorlist[j] = 1 : errorlist[i] = 1;
            //中判断距离，中端优先相信长距离
            else if (armors[i].r_index == armors[j].l_index)
                (getPointsDistance(armors[i].l_light.center, armors[i].r_light.center) <
                    getPointsDistance(armors[j].l_light.center, armors[j].r_light.center)) ?
                errorlist[i] = 1 : errorlist[j] = 1;
            else if (armors[i].l_index == armors[j].r_index)
                (getPointsDistance(armors[i].l_light.center, armors[i].r_light.center) <
                    getPointsDistance(armors[j].l_light.center, armors[j].r_light.center)) ?
                errorlist[i] = 1 : errorlist[j] = 1;
            //右端判距离，右端优先相信短距离
            else if (armors[i].r_index == armors[j].r_index)
                (getPointsDistance(armors[i].l_light.center, armors[i].r_light.center) <
                    getPointsDistance(armors[j].l_light.center, armors[j].r_light.center)) ?
                errorlist[j] = 1 : errorlist[i] = 1;

        }
    }
    //删除游离装甲板|删除倒影装甲板
    for (int errorid=0;errorid < length;errorid++)
        if (errorlist[errorid]){
            vector<size_t>::iterator errhead = errorlist.begin();
            vector<ArmorBox>::iterator armhead = armors.begin();
            armors.erase(armhead + errorid);
            errorlist.erase(errhead+errorid);
            errorid --;
            length--;
        }
}

/*
* @brief: match lights into armors 将识别到的灯条拟合为装甲板
*/
void ArmorDetector::matchArmors() {
    for (int i = 0; i < lights.size()-1 ; i++)
    {
        for (int j = i + 1; j < lights.size(); j++) //just ensure every two lights be matched once 从左至右，每个灯条与其他灯条一次匹配判断
        {
            ArmorBox armor = ArmorBox(lights[i], lights[j]); //construct an armor using the matchable lights 利用左右灯条构建装甲板
            if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
            {
                armor.l_index = i; //set index of left light 左灯条的下标
                armor.r_index = j; //set index of right light 右灯条的下标
                classDeep.getArmorImg(armor);// set armor image 装甲板的二值图
                classDeep.setArmorNum(armor);//set armor number 装甲板数字

                if (armor.type == ArmorType::ERROR_ARMOR)
                {
                    if (armor.armorNum == 1) armor.type = ArmorType::BIG_ARMOR;
                    if (armor.armorNum == 2 || armor.armorNum == 3|| armor.armorNum == 4|| armor.armorNum == 5) { armor.type = ArmorType::SMALL_ARMOR; }
                }

                if(armor.armorNum==0) continue;
                armors.emplace_back(armor); //push into armors 将匹配好的装甲板push入armors中
            }
        }
    }
    if (armors.empty()) {
        state = DetectorState::ARMOR_NOT_FOUND; //if armors is empty then set state ARMOR_NOT_FOUND 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
        return; //exit function
    }
    else {
        eraseErrorRepeatArmor(armors);//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
        if (armors.empty()) {
            state = DetectorState::ARMOR_NOT_FOUND; //if armors is empty then set state ARMOR_NOT_FOUND 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
            return; //exit function
        }
        else {
            state = DetectorState::ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
            return; //exit function
        }
    }
}

/*
 *@brief: set the privious targetArmor as lastArmor and then choose the most valuable armor from current armors as targetArmor
 *			将上一帧的目标装甲板作为lastArmor选择本帧图像中所有装甲板里面价值最大的装甲板作为目标装甲板
 */
void ArmorDetector::setTargetArmor()
{
    if (state == DetectorState::ARMOR_NOT_FOUND)  targetArmor = ArmorBox(); //not found armr then set a default armor as lastArmor 如果状态为没有找到装甲板，则将lastArmor设置为默认的ArmorBox
    else if (state == DetectorState::ARMOR_FOUND) {
        ArmorBox mva = armors[0]; //mva most valuable armor 最适合打击的装甲板
        for (int i = 1; i < armors.size(); i++) //for circle to select the mva 通过遍历装甲板s获取最佳打击装甲板
        {
            if (armorCompare(armors[i], mva, lastArmor, targetNum)) mva = armors[i];
        }
        targetArmor = mva; //set the mva as the targetArmor of this frame
    }
    lastArmor = targetArmor; //first set the targetArmor(of last frame) as lastArmor 将上一帧的targetArmor设置为本帧的lastArmor
}
void ArmorDetector::prerun(Mat& src) {
    //firstly, load and set srcImg  首先，载入并处理图像
    this->setImg(src); //globally srcImg and preprocess it into srcImg_binary 载入Detector的全局源图像 并对源图像预处理成

    srcImg_pre.copyTo(srcImg); //a copy of roiImg, contourImg
    srcImg_binary_pre.copyTo(srcImg_binary); //a copy of roiImg, contourImg
    classDeep.loadImg();
}

/*
 *@brief: an integrative function to run the Detector 集成跑ArmorDetector
 */
void ArmorDetector::daterun() {
    // FPS
    double t = (double)getTickCount();

    //FPS
    //t = (getTickCount() - t) / getTickFrequency();
    //cout<<"wastetime:"<<t*1000<<endl;

    //secondly, reset detector before we findLights or matchArmors(clear lights and armors we found in the last frame and reset the state as LIGHTS_NOT_FOUND)
    //随后，重设detector的内容，清空在上一帧中找到的灯条和装甲板，同时检测器状态重置为LIGHTS_NOT_FOUND（最低状态）
    resetDetector();

    //thirdly, find all the lights in the current frame (srcImg)
    //第三步，在当前图像中找出所有的灯条
    findLights();

    //forthly, if the state is LIGHTS_FOUND (detector found more than two lights) , we match each two lights into an armor
    //第四步，如果状态为LIGHTS_FOUND（找到多于两个灯条），则
    if (state == DetectorState::LIGHTS_FOUND)
    {
        //match each two lights into an armor and if the armor is a suitable one, emplace back it into armors
        //将每两个灯条匹配为一个装甲板，如果匹配出来的装甲板是合适的，则压入armors中
        matchArmors();

        //if the state is ARMOR_FOUND(detector has matched suitable armor), set target armor and last armor
        //如果找到了灯条，则设置好目标装甲板和上一个装甲板
        if (state == DetectorState::ARMOR_FOUND) {
            setTargetArmor();
        }
    }
}




/*
 *@brief: return the Detector status 识别程序是否识别到装甲版
 *@return: FOUND(1) NOT_FOUND(0)
 */
bool ArmorDetector::isFoundArmor()
{
    if (state == DetectorState::ARMOR_FOUND)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 *@:brief: accordingt to the armorNum priority to set the armorScore 根据优先级增加装甲板打击度
 */
void ArmorDetector::setNumScore(const int& armorNum, const int& targetNum, float& armorScore)
{
    if (targetNum == 0)
    {
        if (armorNum == 1) armorScore += 1000;
        else if (armorNum == 2) armorScore += 2000;
        else if (armorNum == 3) armorScore += 3000;
        else if (armorNum == 4) armorScore += 4000;
        else if (armorNum == 5) armorScore += 5000;
        else if (armorNum == 6) armorScore += 6000;
    }
    if (armorNum == targetNum) armorScore += 100000;
}

/*
 *@brief: get the distance of two points(a and b) 获取两点之间的距离
 */
float ArmorDetector::getPointsDistance(const Point2f& a, const Point2f& b) {
    float delta_x = a.x - b.x;
    float delta_y = a.y - b.y;
    //return sqrtf(delta_x * delta_x + delta_y * delta_y);
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}


/*
 *@brief: compare a_armor to b_armor according to their distance to lastArmor(if exit, not a default armor) and their area and armorNum
 *		  比较a_armor装甲板与b_armor装甲板的打击度，判断a_armor是否比b_armor更适合打击（通过装甲板数字是否与目标装甲板数字匹配，装甲板与lastArmor的距离以及装甲板的面积大小判断）
 */
bool ArmorDetector::armorCompare(const ArmorBox& a_armor, const ArmorBox& b_armor, const ArmorBox& lastArmor, const int& targetNum)
{
    float a_score = 0;  // shooting value of a_armor a_armor的打击度
    float b_score = 0;  //shooting value of b_armor b_armor的打击度
    a_score += a_armor.armorRect.area(); //area value of a a_armor面积得分
    b_score += b_armor.armorRect.area(); //area value of b b_armor面积得分

    //number(robot type) priorty 设置a、b装甲板的分数
    setNumScore(a_armor.armorNum, targetNum, a_score);
    setNumScore(b_armor.armorNum, targetNum, b_score);

    if (lastArmor.armorNum != 0) {  //if lastArmor.armorRect is not a default armor means there is a true targetArmor in the last frame 上一帧图像中存在目标装甲板
        float a_distance = getPointsDistance(a_armor.center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
        float b_distance = getPointsDistance(b_armor.center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
        a_score -= a_distance * 2;
        b_score -= b_distance * 2;
    }
    return a_score > b_score; //judge whether a is more valuable according their score 根据打击度判断a是否比b更适合打击
}


///////////////////////////////////////////////////////////  functions  for   debugging      //////////////////////////////////////////////////////////////////
/*
 *@brief: show binary srcImg
 */
void showSrcBinary(Mat& image)
{
    Mat SrcBinaryDisplay;//image for the use of dialaying the lights 显示灯条用的图像
    image.copyTo(SrcBinaryDisplay);//get a copy of srcImg 获取源图像的拷贝
    //if detector finds lights 如果找到了灯条
    if (!SrcBinaryDisplay.empty())
    {
        //show the result image 显示结果图
        imshow("SrcBinaryDisplay", SrcBinaryDisplay);
    }
}
/*
 *@brief: show all the lights found in a copy of srcImg  在图像中显示找到的所有灯条
 */
void showLights(Mat& image, const vector<LightBar>& lights)
{
    Mat lightDisplay;//image for the use of dialaying the lights 显示灯条用的图像
    image.copyTo(lightDisplay);//get a copy of srcImg 获取源图像的拷贝
    //if detector finds lights 如果找到了灯条
    if (!lights.empty())
    {
        putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); //title LIGHT_FOUND 大标题 “找到了灯条”
        for (auto light : lights)
        {
            Point2f lightVertices[4];
            light.lightRect.points(lightVertices);
            //draw all the lights' contours 画出所有灯条的轮廓
            for (size_t i = 0; i < 4; i++)
            {
                line(lightDisplay, lightVertices[i], lightVertices[(i + 1) % 4], Scalar(255, 0, 255), 1, 8, 0);
            }

            //draw the lights's center point 画出灯条中心
            circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

            //show the lights' center point x,y value 显示灯条的中心坐标点
            putText(lightDisplay, to_string(int(light.center.x)), light.center, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, to_string(int(light.center.y)), light.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
        }
    }
    //if detector does not find lights 如果没找到灯条
    else
    {
        putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);//title LIGHT_NOT_FOUND 大标题 “没找到灯条”
    }
    //show the result image 显示结果图
    imshow("Lights Monitor", lightDisplay);
}

/*
 *@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
 */
void showArmors(Mat& image, const vector<ArmorBox>& armors, const ArmorBox& targetArmor)
{
    Mat armorDisplay; //Image for the use of displaying armors 展示装甲板的图像
    image.copyTo(armorDisplay); //get a copy of srcImg 源图像的拷贝
    // if armors is not a empty vector (ARMOR_FOUND) 如果找到了装甲板
    if (!armors.empty())
    {
        putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); //title FOUND 大标题 “找到了装甲板”
        //draw all the armors' vertices and center 画出所有装甲板的顶点边和中心
        for (auto armor : armors)
        {
            //draw the center 画中心
            circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);
            circle(armorDisplay, armorTransmit.tar_predict, 5, Scalar(255, 0, 0), 3);    //predicted point with green

           // 画出所有匹配装甲板
            for (size_t i = 0; i < 4; i++)
            {
                line(armorDisplay, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4], Scalar(255, 255, 255), 2, 8, 0);
            }
            //display its center point x,y value 显示中点坐标
            putText(armorDisplay, to_string(int(armor.center.x)), armor.center, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, to_string(int(armor.center.y)), armor.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, to_string(int(armor.armorNum)), armor.center + Point2f(15, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
        }
        //connect all the vertices to be the armor contour 画出装甲板轮廓
        for (size_t i = 0; i < 2; i++)
        {
            line(armorDisplay, targetArmor.lightVertices[i], targetArmor.lightVertices[(i + 2) % 4], Scalar(255, 255, 0), 2, 8, 0);
        }
    }
    //if armors is a empty vector (ARMOR_NOT FOUND) 如果没找到装甲板
    else
    {
        putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);//title NOT FOUND 大标题 “没找到装甲板”
    }
    line(armorDisplay, Point(0, 239), Point(639, 239), Scalar(0, 0, 255), 1, 8, 0);
    line(armorDisplay, Point(320, 0), Point(320, 480), Scalar(0, 0, 255), 1, 8, 0);
    //show the result armors image 显示结果图
    imshow("Armor Monitor", armorDisplay);
}

/*
 *@brief: show all the lights information in console  在控制台输出找到灯条的中心和角度
 */
void textLights(vector<LightBar>& lights)
{
    cout << "\n################## L I G H T S ##################" << endl;
    if (lights.empty()) {
        cout << "LIGHTS NOT FOUND!" << endl;
    }
    else
    {
        cout << "LIGHTS FOUND!" << endl;
        for (size_t i = 0; i < lights.size(); i++)
        {
            cout << "#############################" << endl;
            cout << "Light Center:" << lights[i].center << endl;
            cout << "Light Angle:" << lights[i].angle << endl;
        }
        cout << "#################################################" << endl;
    }
}

/*
 *@brief: show all the armors information in console  在控制台输出找到装甲板的中心、数字、匹配信息
 */
void textArmors(vector<ArmorBox>& armors)
{
    cout << "\n$$$$$$$$$$$$$$$$$$ A R M O R S $$$$$$$$$$$$$$$$$$" << endl;
    if (armors.empty()) {
        cout << "ARMORS NOT FOUND!" << endl;
    }
    else
    {
        cout << "ARMOR FOUND!" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Armor Number: " << armors[i].armorNum << endl;
            if (armors[i].type == ArmorType::SMALL_ARMOR) cout << "Armor Type: SMALL ARMOR" << endl;
            else if (armors[i].type == ArmorType::BIG_ARMOR) cout << "Armor Type: BIG ARMOR" << endl;
            cout << "\n###### matching information ######" << endl;
            cout << "Angle difference: " << armors[i].getAngleDiff() << endl;
            cout << "Deviation Angle: " << armors[i].getDeviationAngle() << endl;
            cout << "X Dislocation Ration: " << armors[i].getDislocationX() << endl;
            cout << "Y Dislocation Ration: " << armors[i].getDislocationY() << endl;
            cout << "Length Ration: " << armors[i].getLengthRation() << endl;
        }
        cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
    }

}

/*
 *@brief: show all the armors score information in console  在控制台输出找到装甲板的打击度信息
 */
void textScores(vector<ArmorBox>& armors, ArmorBox& lastArmor)
{
    if (!armors.empty())
    {
        cout << "\n@@@@@@@@@@@@@@@@@@ S C O R E S @@@@@@@@@@@@@@@@@@" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            float score = 0;  // shooting value of armor的打击度
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Area: " << armors[i].armorRect.area() << endl;
            score += armors[i].armorRect.area(); //area value of a a_armor面积得分


            if (lastArmor.armorNum != 0) {  //if lastArmor.armorRect is not a default armor means there is a true targetArmor in the last frame 上一帧图像中存在目标装甲板
                float a_distance = 100;// getPointsDistance(armors[i].center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
                cout << "Distance: " << a_distance << endl;
                score -= a_distance * 2;
            }
        }
        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
    }
}

/*
 *@brief: lights, armors, lights to armors every information in one 所有调试用数据输出
 */
void ArmorDetector::showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON, bool textLights_ON, bool textArmors_ON, bool textScores_ON)
{
    if (showSrcImg_ON)
        imshow("srcImg", srcImg);
    if (showSrcBinary_ON)
        showSrcBinary(srcImg_binary);
    if (showLights_ON)
        showLights(srcImg, lights);
    if (showArmors_ON)
        showArmors(srcImg, armors, targetArmor);
    if (textLights_ON)
        textLights(lights);
    if (textArmors_ON)
        textArmors(armors);
    if (textScores_ON)
        textScores(armors, lastArmor);
}

/*
 *@brief: get the vertices and type of target Armor for angle solver 将detector的结果输出
 */
void ArmorDetector::getTargetInfo(vector<Point2f>& lightVertices, Point2f& centerPoint, ArmorType& type)
{
    lightVertices.clear();
    //set traker
    static int detecter_counter=0;设置检测计数器，记录稳定检测次数
    static int tracker_counter=0;
    if(targetArmroTemp.armorNum==0) 
        targetArmroTemp=targetArmor;//Set Traker
    else if(targetArmroTemp.armorNum==targetArmor.armorNum)
    {
        targetArmroTemp = targetArmor;//Update Traker
        detecter_counter++;
    }
    else if(targetArmroTemp.armorNum!=targetArmor.armorNum)
    {
        if(detecter_counter>30)//稳定检测次数超过30，开启5帧追踪器，允许5次检测偏差
        {
            tracker_counter++;// record numbers
            if(tracker_counter > 5)// No records detected 5times,reloading records
            {
                targetArmroTemp = targetArmor;//Update Traker
                tracker_counter = 0;
                detecter_counter = 0;//重新开启稳定检测
            }
            else targetArmroTemp = targetArmroLastTemp;//Update Traker
        } 
        else{
            targetArmroTemp = targetArmor;//Update Traker|未达到稳定检测次数，直接赋值新装甲板
            detecter_counter = 0;//重新开启稳定检测
        }
    }
    else print("error traker,please set again");
    
    targetArmroLastTemp = targetArmroTemp;//record history
    lightVertices = targetArmroTemp.lightVertices;
    centerPoint = targetArmroTemp.center;
    type = targetArmroTemp.type;
}
