#include "ArmorLight.h"


LightBar::LightBar() {
    lightRect = RotatedRect();
    length = 0;
    center = Point2f();
    angle = 0;
}

LightBar::LightBar(const RotatedRect& lightRect,const RotatedRect& miliRect) {
    this->lightRect = lightRect;
    length = MIN(MAX(lightRect.size.height, lightRect.size.width),MAX(miliRect.size.height, miliRect.size.width));
    center = lightRect.center;
    if (lightRect.angle > 90)
        angle = lightRect.angle - 180;
    else
        angle = lightRect.angle;
}

LightBar::~LightBar() {}

ArmorLights armorLights = ArmorLights();
