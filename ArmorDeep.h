#pragma once
#include "ArmorBox.h"
// /**
// * @brief: use warpPerspective to get armorImg and SVM to recognize armorImg 利用透射变换截取装甲板图片（SVM模型大小），并利用SVM来识别装甲板数字
// */
class ArmorDeep
{
public:
    ArmorDeep();
    ~ArmorDeep();

//    /**
//     * @brief: load the SVM model used to recognize armorNum 载入SVM模型（用于识别装甲板数字）
//     * @param: the path of xml_file, the size of the training dataset ImgSize  待载入SVM模型的路径 模型的图片尺寸
//     */
    void loadSvmModel(const char* model_path, Size armorImgSize = Size(64, 64));

//    /**
//     * @brief: load the current roiImage from ArmorDetector 载入roiImage（剪切出装甲板）
//     * @param: the path of xml_file  待载入SVM模型的路径
//     */
    void load_pre(Mat& srcImg_pre);
    void loadImg();

//    /**
//     * @brief: use warpPerspective to get armorImg  利用透视变换获得装甲板图片
//     * @param: the path of xml_file  待载入SVM模型的路径
//     */
    void getArmorImg(ArmorBox& armor);

//    /**
//     * @brief: use SVM to recognize the number of each Armor 利用SVM实现装甲板数字识别
//     */
    void setArmorNum(ArmorBox& armor);

private:
    Ptr<SVM>svm;  //svm model svm模型
    Mat p;		//preRecoginze matrix for svm 载入到SVM中识别的矩阵
    Size armorImgSize; //svm model training dataset size SVM模型的识别图片大小（训练集的图片大小）
    Mat armorImg_pre;//
    Mat warpPerspective_src; //warpPerspective srcImage  透射变换的原图
    Mat warpPerspective_src_pre; //warpPerspective srcImage  透射变换的原图
    Mat warpPerspective_dst; //warpPerspective dstImage   透射变换生成的目标图
    Mat warpPerspective_mat; //warpPerspective transform matrix 透射变换的变换矩阵
    Point2f srcPoints[4];   //warpPerspective srcPoints		透射变换的原图上的目标点 tl->tr->br->bl  左上 右上 右下 左下
    Point2f dstPoints[4];	//warpPerspective dstPoints     透射变换的目标图中的点   tl->tr->br->bl  左上 右上 右下 左下
    HOGDescriptor Hog;
    vector<float> descriptor;
};
