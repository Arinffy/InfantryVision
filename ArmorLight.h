#pragma once
#include "main.h"



/*
 * @brief: information of lightBar besides the armor 装甲板两侧灯条的相关信息
 */
class LightBar
{
public:

    LightBar();
    /*
     *@brief: Parametrical constructor of lightBar 灯条有参构造函数
     *@param: RotatedRect created by fitellipse  拟合椭圆获得的旋转矩形来构造灯条
     */
    LightBar(const RotatedRect& lightRect,const RotatedRect& miliRect) ;
    ~LightBar();

public:
    RotatedRect lightRect;	//rotation rect of light 灯条的旋转矩形（椭圆拟合获得）
    float length;			//length of light bar 灯条长度
    Point2f center;			//center of light bar 灯条中心
    float angle;			//angle of light bar(between length direction and vertical, left 0~90 right 0~-90)
                            //灯条长度方向与竖直方向的夹角，左偏为0~90,右偏为0~-90
};
/*
* @brief: params used in armor-detection  装甲板识别中用到的各种参数
*/
struct ArmorLights {

    int color_threshold;   //color threshold for colorImg from substract channels 通道相减的colorImg使用的二值化阈值
    int bright_threshold;  //color threshold for brightImg 亮度图二值化阈值

    float min_area;		// min area of light bar 灯条允许的最小面积
    float max_angle;	//max angle of light bar 灯条允许的最大偏角

    float max_angle_diff; //max angle difference between two light bars 两个灯条之间允许的最大角度差
    float max_lengthDiff_ratio; //max length ratio difference between two light bars 两个灯条之间允许的最大长度差比值
    float max_deviation_angle; //max deviation angle 两灯条最大错位角

    float max_y_diff_ratio;  //max y y方向投影差比率
    float max_x_diff_ratio;  //max x x方向投影差比率


    //default values  给各参数设定默认值
    ArmorLights() {
        color_threshold = 80;//100
        bright_threshold = 60;//60

        min_area = 50;//50
        max_angle = 45;//45

        max_angle_diff = 8;//6
        max_deviation_angle = 15;//50

        max_x_diff_ratio = 5.5;//4.5
        max_y_diff_ratio = 0.38;//0.5

        max_lengthDiff_ratio = 0.27;//0.5
    }
};
extern ArmorLights armorLights;
