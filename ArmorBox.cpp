#include "ArmorBox.h"

/*
 *@brief: calculate the cross point of four points in order bl(below left),tl(top left),tr(top right),br(below right)
 */
const Point2f crossPointof(const Point2f& bl, const Point2f& tl, const Point2f& tr, const Point2f& br) {
    float a1 = tr.y - bl.y;
    float b1 = tr.x - bl.x;
    float c1 = bl.x * tr.y - tr.x * bl.y;

    float a2 = br.y - tl.y;
    float b2 = br.x - tl.x;
    float c2 = tl.x * br.y - br.x * tl.y;

    float d = a1 * b2 - a2 * b1;

    if (d == 0.0) {
        return Point2f(FLT_MAX, FLT_MAX);
    }
    else {
        return cv::Point2f((b2 * c1 - b1 * c2) / d, (c1 * a2 - c2 * a1) / d);
    }
}

/*
 *@brief: using the lightRect of two lightBar to construct the armorVertices
 */
void setArmorVertices(const LightBar& l_light, const LightBar& r_light, ArmorBox& armor) {

    //light points
    armor.lightVertices[0] = l_light.angle > 0 ?
        Point2f(l_light.center.x + 0.5 * l_light.length * sin(l_light.angle / 57),
            l_light.center.y - 0.5*l_light.length * cos(l_light.angle / 57)):
        Point2f(l_light.center.x - 0.5 * l_light.length * cos(0.5*CV_PI + l_light.angle / 57),
            l_light.center.y - 0.5 * l_light.length * sin(0.5 * CV_PI + l_light.angle / 57));

    armor.lightVertices[1] = r_light.angle > 0 ?
        Point2f(r_light.center.x + 0.5 * r_light.length * sin(r_light.angle / 57),
            r_light.center.y - 0.5 * r_light.length * cos(r_light.angle / 57)) :
        Point2f(r_light.center.x - 0.5 * r_light.length * cos(0.5 * CV_PI + r_light.angle / 57),
            r_light.center.y - 0.5 * r_light.length * sin(0.5 * CV_PI + r_light.angle / 57));

    armor.lightVertices[2] = r_light.angle > 0 ?
        Point2f(r_light.center.x - 0.5 * r_light.length * sin(r_light.angle / 57),
            r_light.center.y + 0.5 * r_light.length * cos(r_light.angle / 57)) :
        Point2f(r_light.center.x + 0.5 * r_light.length * cos(0.5 * CV_PI + r_light.angle / 57),
            r_light.center.y + 0.5 * r_light.length * sin(0.5 * CV_PI + r_light.angle / 57));

    armor.lightVertices[3] = l_light.angle > 0 ?
        Point2f(l_light.center.x - 0.5 * l_light.length * sin(l_light.angle / 57),
            l_light.center.y + 0.5 * l_light.length * cos(l_light.angle / 57)) :
        Point2f(l_light.center.x + 0.5 * l_light.length * cos(0.5 * CV_PI + l_light.angle / 57),
            l_light.center.y + 0.5 * l_light.length * sin(0.5 * CV_PI + l_light.angle / 57));

    //armor points|armor length is double with light length

    armor.armorVertices[0] = l_light.angle > 0 ?
        Point2f(l_light.center.x + 0.5 * 2*l_light.length * sin(l_light.angle / 57),
            l_light.center.y - 0.5*2*l_light.length * cos(l_light.angle / 57)):
        Point2f(l_light.center.x - 0.5 * 2*l_light.length * cos(0.5*CV_PI + l_light.angle / 57),
            l_light.center.y - 0.5 * 2*l_light.length * sin(0.5 * CV_PI + l_light.angle / 57));

    armor.armorVertices[1] = r_light.angle > 0 ?
        Point2f(r_light.center.x + 0.5 * 2*r_light.length * sin(r_light.angle / 57),
            r_light.center.y - 0.5 * 2*r_light.length * cos(r_light.angle / 57)) :
        Point2f(r_light.center.x - 0.5 * 2*r_light.length * cos(0.5 * CV_PI + r_light.angle / 57),
            r_light.center.y - 0.5 * 2*r_light.length * sin(0.5 * CV_PI + r_light.angle / 57));

    armor.armorVertices[2] = r_light.angle > 0 ?
        Point2f(r_light.center.x - 0.5 * 2*r_light.length * sin(r_light.angle / 57),
            r_light.center.y + 0.5 * 2*r_light.length * cos(r_light.angle / 57)) :
        Point2f(r_light.center.x + 0.5 * 2*r_light.length * cos(0.5 * CV_PI + r_light.angle / 57),
            r_light.center.y + 0.5 * 2*r_light.length * sin(0.5 * CV_PI + r_light.angle / 57));

    armor.armorVertices[3] = l_light.angle > 0 ?
        Point2f(l_light.center.x - 0.5 * 2*l_light.length * sin(l_light.angle / 57),
            l_light.center.y + 0.5 * 2*l_light.length * cos(l_light.angle / 57)) :
        Point2f(l_light.center.x + 0.5 * 2*l_light.length * cos(0.5 * CV_PI + l_light.angle / 57),
            l_light.center.y + 0.5 * 2*l_light.length * sin(0.5 * CV_PI + l_light.angle / 57));
}


ArmorBox::ArmorBox() {
    l_index = -1;
    r_index = -1;
    l_light = LightBar();
    r_light = LightBar();
    armorNum = 0;
    lightVertices.resize(4);
    armorVertices.resize(4);
    type = ArmorType::SMALL_ARMOR;
    center = Point2f();
    armorRect = Rect();
    armorImg = Mat();
}

ArmorBox::ArmorBox(const LightBar& l_light, const LightBar& r_light) {
    this->l_light = l_light;
    this->r_light = r_light;

    armorNum = 0;
    armorAngle = (l_light.angle + r_light.angle) / 2;

    //set armorVertices tl->tr->br->bl     ���� ���� ���� ����
    lightVertices.resize(4);
    armorVertices.resize(4);
    setArmorVertices(l_light, r_light, *this);    // '*this' means the reference of this ArmorBox

    //set armor center
    center = crossPointof(lightVertices[0], lightVertices[1], lightVertices[2], lightVertices[3]);

    //set armorRect using boundingRect for convenience
    armorRect = boundingRect(lightVertices);

    type = getArmorclass(l_light, r_light);
}

ArmorBox::~ArmorBox() {}
ArmorType ArmorBox::getArmorclass(const LightBar& l_light, const LightBar& r_light)
{
    ArmorType getType;

    if (this->getDislocationX() < 3.1&& this->getDislocationX() > 2.1)//2.6
        getType = ArmorType::SMALL_ARMOR;
    else if(this->getDislocationX() < 5.06 && this->getDislocationX() > 3.46)//4.56
        getType = ArmorType::BIG_ARMOR;
    else getType = ArmorType::ERROR_ARMOR;

    return getType;
}
// angle difference: the angle difference of left and right lights 装甲板左右灯条角度差
float ArmorBox::getAngleDiff() const {
    float angle_diff = abs(l_light.angle - r_light.angle); //get the abs of angle_diff 灯条的角度差
    return angle_diff;
}

// deviation angle : the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)
float ArmorBox::getDeviationAngle() const {
    float delta_x = r_light.center.x - l_light.center.x; //Δx
    float delta_y = r_light.center.y - l_light.center.y; //Δy
    float deviationAngle = fabs(atan(delta_y / delta_x)) * 180.0f / CV_PI; //tanθ=Δy/Δx
    return deviationAngle;
}

// dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
float ArmorBox::getDislocationX() const {
    float meanLen = (l_light.length + r_light.length) / 2;
    float xDiff = abs(l_light.center.x - r_light.center.x); //x distance ration y轴方向上的距离比值（y轴距离与灯条平均值的比）
    float xDiff_ratio = xDiff / meanLen;
    return xDiff_ratio;
}

// dislocation judge Y:  r-l light center distance ration on the Y-axis 灯条位置差距 两灯条中心Y方向差距比值
float ArmorBox::getDislocationY() const {
    float meanLen = (l_light.length + r_light.length) / 2;
    float yDiff = abs(l_light.center.y - r_light.center.y);  //y distance ration x轴方向上的距离比值（x轴距离与灯条平均值的比）
    float yDiff_ratio = yDiff / meanLen;
    return yDiff_ratio;
}

// length difference ration: the length difference ration r-l lights 左右灯条长度差比值
float ArmorBox::getLengthRation() const {
    float length_diff = abs(l_light.length - r_light.length);
    float lengthDiffRation = length_diff / MAX(l_light.length, r_light.length);
    return lengthDiffRation;
}

// judge whether this armor is suitable or not  判断本装甲板是否是合适的装甲板
bool ArmorBox::isSuitableArmor() const
{
        cout << "center:" << l_light.center << "," << r_light.center << endl;
        cout << "angle_deff:" << this->getAngleDiff() << endl;
        cout << "deviationAngle:" << this->getDeviationAngle() << endl;
        cout << "xDiff_ratio:" << this->getDislocationX() << endl;
        cout << "yDiff_ratio:" << this->getDislocationY() << endl;
        cout << "lengthDiffRation:" << this->getLengthRation() << endl;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if(armorReceive.gyro_pitch>0&&0.5*(l_light.center.y+r_light.center.y)<450)//wait for picture size
        return
        this->getAngleDiff() < armorLights.max_angle_diff &&		// angle difference judge the angleDiff should be less than max_angle_diff �����ǶȲ��жϣ���С�������������ǲ�
        this->getDeviationAngle() < 1.5*armorLights.max_deviation_angle &&		// deviation angle judge: the horizon angle of the line of centers of lights ������λ�Ƚ�(����������������ˮƽ�߼н�)�ж�
        this->getDislocationX() < armorLights.max_x_diff_ratio &&		// dislocation judge: the x and y can not be too far ����λ�ò��� ����������x��y�������಻��ƫ�����ñ�ֵ��Ϊ�������ݣ�
        this->getDislocationY() < 2*armorLights.max_y_diff_ratio &&		// dislocation judge: the x and y can not be too far ����λ�ò��� ����������x��y�������಻��ƫ�����ñ�ֵ��Ϊ�������ݣ�
        this->getLengthRation() < armorLights.max_lengthDiff_ratio;
    else return
        this->getAngleDiff() < armorLights.max_angle_diff &&		// angle difference judge the angleDiff should be less than max_angle_diff 灯条角度差判断，需小于允许的最大角差
        this->getDeviationAngle() < armorLights.max_deviation_angle &&		// deviation angle judge: the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)判断
        this->getDislocationX() < armorLights.max_x_diff_ratio &&		// dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
        this->getDislocationY() < armorLights.max_y_diff_ratio &&		// dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
        this->getLengthRation() < armorLights.max_lengthDiff_ratio;
}
