#pragma once
#include "main.h"
#include "ArmorLight.h"

/*
 * @brief: information of Armor 装甲板相关数据信息
 */
class ArmorBox
{
public:
    ArmorBox();
    /*
     *@brief: Parametrical constructor of armorBox 装甲板有参构造函数
     *@param: two LightBar  左右两个灯条
     */
    ArmorBox(const LightBar& l_light, const LightBar& r_light);
    ~ArmorBox();
    ArmorType getArmorclass(const LightBar& l_light, const LightBar& r_light);

    // angle difference: the angle difference of left and right lights 装甲板左右灯条角度差
    float getAngleDiff() const;

    // deviation angle : the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)
    float getDeviationAngle() const;

    // dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
    float getDislocationX() const;

    // dislocation judge Y:  r-l light center distance ration on the Y-axis 灯条位置差距 两灯条中心Y方向差距比值
    float getDislocationY() const;

    // length difference ration: the length difference ration r-l lights 左右灯条长度差比值
    float getLengthRation() const;

    // an integrative function to judge whether this armor is suitable or not
    bool isSuitableArmor() const;

public:
    LightBar l_light, r_light; //the left and right lightbar of this armor 装甲板的左右灯条
    int l_index, r_index; //the index of left and right light 左右灯条的下标(默认为-1，仅作为ArmorDetector类成员时生效)
    int armorNum;  //number on armor(recognized by SVM) 装甲板上的数字（用SVM识别得到）
    vector<Point2f> lightVertices;  // bl->tl->tr->br     左下 左上 右上 右下
    vector<Point2f> armorVertices;  // bl->tl->tr->br     左下 左上 右上 右下
    ArmorType type; //the type of armor
    Point2f center;	// center point(crossPoint) of armor 装甲板中心
    Rect armorRect;  //armorRect for roi 装甲板的矩形获取roi用
    float armorAngle;//armor angle(mean of lightBars) 装甲板角度(灯条角度的平均值)
    Mat armorImg;	//image of armor set by getArmorImg() from ArmorNumClassifier() 装甲板的图片（透射变换获得）
};
