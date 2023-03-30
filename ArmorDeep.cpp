#include"ArmorDeep.h"
int ammer_num;
ArmorDeep::ArmorDeep() {
    svm = ml::SVM::create();
    armorImgSize = Size(64, 64);
    p = Mat();
    Hog= HOGDescriptor(Size(64,64),Size(16,16),Size(8,8),Size(8,8),9);
    warpPerspective_mat = Mat(3, 3, CV_32FC1);
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(armorImgSize.width, 0);
    dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
    dstPoints[3] = Point2f(0, armorImgSize.height);
}

ArmorDeep::~ArmorDeep() {}

void ArmorDeep::loadSvmModel(const char* model_path, Size armorImgSize) {
    svm = StatModel::load<SVM>(model_path);
    if (svm.empty())
    {
        cout << "Svm load error! Please check the path!" << endl;
        exit(0);
    }
    this->armorImgSize = armorImgSize;

    //set dstPoints (the same to armorImgSize, as it can avoid resize armorImg)
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(armorImgSize.width, 0);
    dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
    dstPoints[3] = Point2f(0, armorImgSize.height);
}
void ArmorDeep::load_pre(Mat& srcImg_pre) {

    //copy srcImg as warpPerspective_src
//    (srcImg_pre).copyTo(warpPerspective_src_pre);
    (srcImg_pre).copyTo(armorImg_pre);
    //preprocess srcImg for the goal of acceleration
//    cvtColor(warpPerspective_src_pre, warpPerspective_src_pre, 6);  //CV_BGR2GRAY=6
//    threshold(warpPerspective_src_pre, warpPerspective_src_pre, 30, 255, THRESH_BINARY);
    //medianBlur(warpPerspective_src_pre,warpPerspective_src_pre,9);
}
void ArmorDeep::loadImg() {
    (armorImg_pre).copyTo(warpPerspective_src);
}

void ArmorDeep::getArmorImg(ArmorBox& armor) {
    //set the armor vertex as srcPoints
    for (int i = 0; i < 4; i++)
        srcPoints[i] = armor.armorVertices[i];

    //get the armor image using warpPerspective
    warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  透射变换矩阵
    warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize, INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage
    warpPerspective_dst.copyTo(armor.armorImg); //copyto armorImg
//    imshow("armorImg",armor.armorImg);
//    waitKey(2);
}

void ArmorDeep::setArmorNum(ArmorBox& armor) {

    // adapt armorImg to the SVM model sample-size requirement
    p = armor.armorImg;

    resize(p,p,Size(64,64));
    cvtColor(p,p,COLOR_BGR2GRAY);
//    imshow("p",p);
    waitKey(2);

    Hog.compute(p,descriptor,Size(1,1),Size(0,0));
    Mat testDescriptor=Mat::zeros(1,descriptor.size(),CV_32FC1);
    for(size_t i=0;i<descriptor.size();i++)
        testDescriptor.at<float>(0,i)=descriptor[i];
//    p.convertTo(p, CV_32FC1);

    //set armor number according to the result of SVM
    armor.armorNum = (int)svm->predict(testDescriptor);
    ammer_num=armor.armorNum;

//if(armor.armorNum!=7){
//    static int i=0;
//    stringstream str;
//    str<<"4_"<<i<<".jpg";
//    imwrite("/home/cod-1th/Desktop/7/"+str.str(),p);
//    i++;}
//    cout<<"armorNum:"<<armor.armorNum<<endl;
}
