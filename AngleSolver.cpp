/*
*	@Author: USTL-COD
*	@Date:	 2022.04.04
*	@Brief:  This header file declares all the classes and params used to solve eular angle
*/

#include "AngleSolver.h"
#include "ArmorBox.h"
#include "ArmorDeep.h"
#include "ArmorDetector.h"
 extern int ammer_num;
AngleSolver::AngleSolver()
{
    //Set angle solver prop
    setCameraParam("/home/cod-1th/Desktop/file_calibration/HK_2_0415_640x480.xml", 1);
    setArmorSize(ArmorType::SMALL_ARMOR, 135, 60);//120
    setArmorSize(ArmorType::BIG_ARMOR, 230, 60);//4.2

    //dt = 16;//ms  processdelay+firedelay+serialdelay+flydelay
    measurement = Mat::zeros(measureNum, 1, CV_32F);
    controlment = Mat::zeros(controlNum-2, 1, CV_32F);
    tarpredict = Point2f(320,240);//画幅中心，记得修改

    KF.measurementMatrix = (Mat_<float>(measureNum, measureNum)
        <<  1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1);		//测量矩阵 H=[1,0,0,0;0,1,0,0]

    KF.processNoiseCov = (Mat_<float>(stateNum, stateNum)
      <<  1e-6,0, 0, 0, 0,
          0, 3e-12, 0, 0, 0,
          0, 0, 1e-6,0, 0,
          0, 0, 0, 1e-15, 0,
          0, 0, 0, 0, 1e-10);	//系统噪声方差矩阵Q，高斯白噪声，单位阵

    KF.measurementNoiseCov = (Mat_<float>(stateNum, stateNum)
        << 8.834e-08, 0, 0, 0, 0,
            0, 5.197e-10, 0, 0, 0,
            0, 0, 1.132e-07, 0, 0,
            0, 0, 0, 2.44273e-09, 0,
            0, 0, 0, 0, 1e-05);	//测量噪声方差矩阵R，高斯白噪声，单位阵

    KF.errorCovPost = (Mat_<float>(stateNum, stateNum)
        <<  1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1);	//后验错误估计协方差矩阵P，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));//初始化状态为随机值

    target_spinning_ = false;
    jump_period_ = 0.0;
    jump_count_ = 0;
    last_jump_time_ = 0.0;
    last_jump_position_ = Point2f(0.0, 0.0);
    max_jump_angle = 0.007;
    max_jump_period = 0.8;
    allow_following_range = 0.3;
    projectileInit(0,-30,70,-1.6,-0.2);//220 230 0 -4 -2.85
    cout << "AngleSolver is running" << endl;

}

AngleSolver::~AngleSolver()
{
    cout << "AngleSolver is finished" << endl;
}

int AngleSolver::setCameraParam(const char * filePath, int camId)
{
    FileStorage fsRead;
    fsRead.open(filePath, FileStorage::READ);
    if (!fsRead.isOpened())
    {
        cout << "Failed to open xml" << endl;
        return -1;
    }

    switch (camId)
    {
    case 1:
        fsRead["camera_matrix"] >> CAMERA_MATRIX;
        fsRead["distortion_coefficients"] >> DISTORTION_COEFF;
        break;
    default:
        cout << "WRONG CAMID GIVEN!" << endl;
        break;
    }
    fsRead.release();
    return 0;
}

void AngleSolver::setArmorSize(ArmorType type, double width, double height)
{
    double half_x = width / 2.0;
    double half_y = height / 2.0;
    switch (type)
    {
    case ArmorType::SMALL_ARMOR:
        SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0));   //tl top left
        SMALL_ARMOR_POINTS_3D.push_back(Point3f( half_x, -half_y, 0));	//tr top right
        SMALL_ARMOR_POINTS_3D.push_back(Point3f( half_x,  half_y, 0));   //br below right
        SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x,  half_y, 0));  //bl below left
        break;

    case ArmorType::BIG_ARMOR:
        BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0));   //tl top left
        BIG_ARMOR_POINTS_3D.push_back(Point3f( half_x, -half_y, 0));    //tr top right
        BIG_ARMOR_POINTS_3D.push_back(Point3f( half_x,  half_y, 0));    //br below right
        BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x,  half_y, 0));   //bl below left
        break;
    default: break;
    }
}

void AngleSolver::setBulletSpeed(int bulletSpeed)
{
    bullet_speed = bulletSpeed;
}
void AngleSolver::setGyroYaw(float yaw)
{
    gyro_yaw = yaw;
}
void AngleSolver::setGyroPitch(float pitch)
{
    gyro_pitch = pitch;
}
void AngleSolver::setTarget(vector<Point2f> contourPoints, Point2f centerPoint, ArmorType type)
{
    targetContour = contourPoints;
    targetCenter = centerPoint;
    targetType = type;
}

void AngleSolver::solveAngles()
{
    Mat _rvec;
    switch (targetType)
    {
    case ArmorType::SMALL_ARMOR:
        solvePnP(SMALL_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, _rvec, tVec, false, SOLVEPNP_ITERATIVE);
        cout<<"small"<<endl;
        break;
    case ArmorType::BIG_ARMOR:
        solvePnP(BIG_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, _rvec, tVec, false, SOLVEPNP_ITERATIVE);
        cout<<"big"<<endl;
        break;
    default:
        cout<<"?";
//        return;
        break;
    }
//    cout<<"tar:"<<targetContour<<endl;
    x_pos = (tVec.at<double>(0, 0)+offset_x)/1000;
    y_pos = (tVec.at<double>(1, 0)+offset_y)/1000;
    z_pos = (tVec.at<double>(2, 0)+offset_z)/1000;
    distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    cout<<"tVex:"<<x_pos<<" "<<y_pos<<" "<<z_pos<<endl;
    // Target is too far, using PinHole solver
    if (distance > 9)
    {
        PinHole_solver();
    }
    // Target is moderate, using PnP solver
    else
    {
        P4P_solver();
    }
}

void AngleSolver::P4P_solver()
{
//    cout<<"88888888888888888888888888"<<ammer_num<<endl;
    if(ammer_num==7&&armorReceive.predict==0 )
    y_pitch = -atan2(y_pos,sqrt(z_pos*z_pos+x_pos*x_pos)) + (float)(offset_pitch+1.7)*CV_PI/180;
    else
    y_pitch = -atan2(y_pos,sqrt(z_pos*z_pos+x_pos*x_pos)) + (float)(offset_pitch)*CV_PI/180;
    x_yaw = atan2(x_pos,z_pos) + (float)(offset_yaw)*CV_PI/180;
}

void AngleSolver::PinHole_solver()
{
    double fx = CAMERA_MATRIX.at<double>(0, 0);
    double fy = CAMERA_MATRIX.at<double>(1, 1);
    double cx = CAMERA_MATRIX.at<double>(0, 2);
    double cy = CAMERA_MATRIX.at<double>(1, 2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(targetCenter);
    //对像素点去畸变
    undistortPoints(in, out, CAMERA_MATRIX, DISTORTION_COEFF, noArray(), CAMERA_MATRIX);
    pnt = out.front();
    //57
//    x_yaw = atan2((pnt.x - cx) ,fx) / CV_PI * 180 + (float)(offset_yaw);
//    y_pitch = -atan2((pnt.y - cy) , fy ) / CV_PI * 180 + (float)(offset_pitch);//add angle pitch
}

void AngleSolver::updateKFParam(const double& delt_time)
{
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum)
     <<  1,delt_time, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1,delt_time, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1);     //状态转移矩阵A  ，控制矩阵B默认为零

    KF.controlMatrix = (Mat_<float>(controlNum, controlNum - 2)
        <<  0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0);		//控制矩阵

    controlment = (Mat_<float>(controlNum - 2, 1)
        << evaluateact[2], evaluateact[3], distance / delt_time);

    measurement= (Mat_<float>(measureNum, 1)
        << x_yaw+gyro_yaw, evaluateact[0],y_pitch+gyro_pitch, evaluateact[1], distance );
}
Vec4f AngleSolver::getAngularv(const double& now_delt_time)
{
    static double last_delt_time = now_delt_time;
    static Point2f last_relative_angle=Point2f(x_yaw+gyro_yaw,y_pitch+gyro_pitch);
    static Point2f last_delt_angle=Point2f(x_yaw+gyro_yaw- last_relative_angle.x,y_pitch+gyro_pitch-last_relative_angle.y);
    //static Point2f last_angular_vel = Point2f(0,0);

    Point2f now_delt_angle = Point2f(x_yaw+gyro_yaw - last_relative_angle.x, y_pitch+gyro_pitch - last_relative_angle.y);

    now_angular_vel = Point2d(  (last_delt_time * last_delt_time * now_delt_angle.x + now_delt_time * now_delt_time * last_delt_angle.x) /
                                        (last_delt_time * now_delt_time * now_delt_time + last_delt_time * last_delt_time * now_delt_time),
                                        (last_delt_time * last_delt_time * now_delt_angle.y + now_delt_time * now_delt_time * last_delt_angle.y) /
                                        (last_delt_time * now_delt_time * now_delt_time + last_delt_time * last_delt_time * now_delt_time));
    now_angular_acc = Point2d(  2 * (last_delt_time * now_delt_angle.x - now_delt_time * last_delt_angle.x ) /
                                        (last_delt_time * now_delt_time * now_delt_time + last_delt_time * last_delt_time * now_delt_time),
                                        2 * (last_delt_time * now_delt_angle.y - now_delt_time * last_delt_angle.y ) /
                                        (last_delt_time * now_delt_time * now_delt_time + last_delt_time * last_delt_time * now_delt_time));
    //Point2f now_angular_vel = Point2d(
    //    (2*last_delt_angle.x-last_delt_time*last_angular_vel.x)/last_delt_time,
    //    (2*last_delt_angle.y-last_delt_time*last_angular_vel.y)/last_delt_time);
    //Point2f now_angular_acc = Point2d(
    //    2*(last_delt_time*now_delt_time*last_angular_vel.x-2*last_delt_angle.x*now_delt_time+now_delt_angle.x*last_delt_time)/
    //    (last_delt_time*now_delt_time*now_delt_time),
    //    2*(last_delt_time*now_delt_time*last_angular_vel.y-2*last_delt_angle.y*now_delt_time+now_delt_angle.y*last_delt_time) /
    //    (last_delt_time * now_delt_time * now_delt_time));

    cout << "last_relative_angle:" << last_relative_angle << endl;
    cout << "now_deltang:" << now_delt_angle << endl;
    cout << "now_angular_vel:" << now_angular_vel << endl;
    cout << "now_angular_acc:" << now_angular_acc << endl;
    last_delt_time = now_delt_time;
    last_relative_angle = Point2f(x_yaw+gyro_yaw, y_pitch+gyro_pitch);
    last_delt_angle = now_delt_angle;
    //last_angular_vel = now_angular_vel;

    Vec4f now_evaluate;
    now_evaluate[0] = now_angular_vel.x;
    now_evaluate[1] = now_angular_vel.y;
    now_evaluate[2] = now_angular_acc.x;
    now_evaluate[3] = now_angular_acc.y;

    return now_evaluate;
}

void AngleSolver::anglePredict()
{
    static double time_last=0;
    static double time_delt=13;//set time
    static double max_match_distance_=0.8;//最大预测距离差
    time_delt=timecolck-time_last;
    time_last=timecolck;

    time_delt > 25 ? time_delt=13 : time_delt;
    cout << "wasteime_date:" << time_delt << endl;
   if(time_delt)
   {
       evaluateact=getAngularv(time_delt);
       updateKFParam(time_delt);
       //KF.predict();//预测矩阵
       Mat predict_msg = KF.predict(controlment);//预测矩阵
       Mat predict_position = (Mat_<float>(3 , 1)
                               << predict_msg.at<float>(0)*10,predict_msg.at<float>(2)*10,predict_msg.at<float>(4)*1);
       Mat diff_of_predict;
       Mat Measurement=(Mat_<float>(3,1)
                        <<measurement.at<float>(0)*10,measurement.at<float>(2)*10,measurement.at<float>(4)*1);
       absdiff(predict_position,Measurement,diff_of_predict);
       double position_dif=norm(diff_of_predict);
       cout<<"***********position diff:"<<position_dif<<endl;
       if(position_dif > max_match_distance_)
       {
           KF.statePost.at<float>(0)= x_yaw+gyro_yaw;
           KF.statePost.at<float>(1)= 0;
           KF.statePost.at<float>(2)= y_pitch+gyro_pitch;
           KF.statePost.at<float>(3)= 0;
           predictangle=Point2f(x_yaw+gyro_yaw,y_pitch+gyro_pitch);
           predictcenter = Point2f(CAMERA_MATRIX.at<double>(0, 2)+1.02*tan(predictangle.x -gyro_yaw)* CAMERA_MATRIX.at<double>(0, 0),
               CAMERA_MATRIX.at<double>(1, 2)-1.02*tan(predictangle.y-gyro_pitch)* CAMERA_MATRIX.at<double>(1, 1));
           predictmatrix = KF.correct(measurement);
           return;
       }
       predictmatrix = KF.correct(measurement);     //更新测量值：输入形参：实际输出z(k)，求出了卡尔曼增益Kk、后验更新状态向量x(k) 和 后验更新协方差矩阵p(k)


       predictangularv = Point2f(predictmatrix.at<float>(1), predictmatrix.at<float>(3));

       predictangle = Point2f(predictmatrix.at<float>(0), predictmatrix.at<float>(2))+ predictangularv*(1000*flytime+time_delt);//1000*flytime+time_delt+25
       cout << "diffangle:" << predictangle.x- (gyro_yaw)<<","<< predictangle.y-(gyro_pitch) <<","<<predictangle << endl;
       predictcenter = Point2f(CAMERA_MATRIX.at<double>(0, 2)+1.02*tan(predictangle.x -gyro_yaw)* CAMERA_MATRIX.at<double>(0, 0),
       CAMERA_MATRIX.at<double>(1, 2)-1.02*tan(predictangle.y-gyro_pitch)* CAMERA_MATRIX.at<double>(1, 1));  //预测值(x',y')
//       cout << "predictmatrix:" << predictmatrix << endl;
//       cout << "measurement:" << measurement << endl;
//       cout << "predictangle:" << (1000*flytime+time_delt+25) << endl;
//       cout << "predictcenter:" << predictcenter << endl;
//       cout << "len:" << predictmatrix.at<float>(4) << endl;
       tarpredict = targetCenter;
   }

}

void AngleSolver::Spinobserver()
{
    double current_time = timecolck;
    double time_after_jumping = (current_time - last_jump_time_);
//    cout<< "**************time_after_jumping"<<time_after_jumping<<endl;
    if (time_after_jumping > max_jump_period) {
      target_spinning_ = false;
      jump_count_ = 0;
    }

    double current_yaw = 0.0;
    double yaw_diff = 0.0;
    if (track_state) {
        track_state=LOST;
      current_yaw = x_yaw+gyro_yaw;
      yaw_diff = fmod(current_yaw-last_yaw_+M_PI,2.0*M_PI);
      yaw_diff = yaw_diff<=0.0 ? yaw_diff+M_PI : yaw_diff-M_PI;
        cout<<"***********yaw_diff"<<yaw_diff<<endl;

      if (std::abs(yaw_diff) > max_jump_angle) {
        jump_count_++;
        if (jump_count_ > 1 && std::signbit(yaw_diff) == std::signbit(last_jump_yaw_diff_)) {
          target_spinning_ = true;
          jump_period_ = time_after_jumping;
        }

        last_jump_time_ = current_time;
        last_jump_position_ = Point2f(x_yaw,y_pitch);
        last_jump_yaw_diff_ = yaw_diff;
      }

      last_yaw_ = current_yaw;
    }

    if (target_spinning_) {
      if (time_after_jumping / jump_period_ < allow_following_range) {
//        target_msg.suggest_fire = true;
      } else {
        x_yaw = last_jump_position_.x;
        y_pitch = last_jump_position_.y;
        KF.statePost.at<float>(1)= 0;
        KF.statePost.at<float>(3)= 0;
//        target_msg.suggest_fire = false;
      }
    }
//    else {
//      target_msg.suggest_fire = target_msg.tracking;
//    }

    // Update spin_info_msg
//    spin_info_msg.header = target_msg.header;
//    spin_info_msg.target_spinning = target_spinning_;
//    spin_info_msg.suggest_fire = target_msg.suggest_fire;
//    spin_info_msg.jump_count = jump_count_;
//    spin_info_msg.yaw_diff = yaw_diff;
//    spin_info_msg.jump_period = jump_period_;
//    spin_info_msg.time_after_jumping = time_after_jumping;
}

void AngleSolver::projectileInit(float x,float y,float z,float pitch,float yaw)
{
    offset_x=x;
    offset_y=y;
    offset_z=z;
    offset_pitch=pitch;
    offset_yaw=yaw;

    friction_factor=0.47;//0.47
    friction_density=1.169;//0-1.293 25-1.169 kg/m^3
    friction_radius=0.017/2;//0.017 0.042
    friction_weight=0.0032;//0.00315 0.0408
    friction_contact=2*CV_PI*friction_radius*friction_radius;//All:4 Right:2
    friction_cure=0.5*friction_factor*friction_density*friction_contact/friction_weight;//the end factor
    gravity=9.8;
}

float AngleSolver::projectileModel(float tarx,float bollv,float pitch)
{// mm mm/s rad
    flytime=(float)((exp(friction_cure*tarx)-1)/(friction_cure*bollv*cos(pitch)));
    float diffdis=(float)(bollv*sin(pitch)*flytime-gravity*flytime*flytime/2);
    cout<<"time:"<<flytime<<endl;
    return diffdis;
}

float AngleSolver::projectileGet(float tarx,float tary,float bollv)
{
    float tempy, actualy, diffy;
    float tempangle;
    tempy = tary;
    // by iteration
    for (int i = 0; i < 20; i++) {
        tempangle = (float) atan2(tempy, tarx);
        actualy = projectileModel(tarx, bollv, tempangle);
        diffy = tary - actualy;
        tempy = tempy + diffy;
        printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,tempangle*180/3.1415926535,tempy,diffy);
        if (fabsf(diffy) < 0.01)
        break;
    }

    return tempangle-(float) atan2(tary, tarx);
    //return (float) atan2(tempy+tary, tarx)*180/CV_PI;
}

void AngleSolver::projectileTransform(float& pitch)
{
    float pitch1 =projectileGet(distance*cos(gyro_pitch+pitch*CV_PI/180), distance*sin(gyro_pitch+pitch*CV_PI/180), bullet_speed);
//    cout<<"dea pitch:"<<pitch1<<endl;
//        cout<<"$$$$$$ speed:"<<bullet_speed<<endl;

    y_pitch+=pitch1;//+2*CV_PI/180
//    cout<<"ppppitch1:"<<y_pitch;
}


void AngleSolver::getAngle(vector<Point2f>& contourPoints,Point2f cenerPoint, ArmorType type, double & yaw, double & pitch, double & evaluateDistance,Point2f& precenter)
{
    track_state=TRACKING;
    setTarget(contourPoints, cenerPoint, type);
    solveAngles();
    projectileTransform(y_pitch);
    Spinobserver();
    anglePredict();

    yaw = 0.9*(predictangle.x-gyro_yaw)*180/CV_PI;
    pitch = (y_pitch)*180/CV_PI;//+0.55*3.8
//    yaw = x_yaw;
//    pitch = y_pitch;


//    if(now_angular_vel.x*now_angular_acc.x < 0)
//    {
//        yaw = (x_yaw*180/CV_PI)*0.7+0.3*(predictangle.x-gyro_yaw)*180/CV_PI;
//    }

    evaluateDistance = distance;
    precenter = predictcenter;
}

void AngleSolver::showDebugInfo(bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams)
{
    if(showCurrentResult)
    {
        Mat angleImage = Mat::zeros(250,600,CV_8UC3);
        putText(angleImage, "Yaw: " + to_string(x_yaw), Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Pitch: " + to_string(y_pitch), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Distance: " + to_string(distance), Point(100, 150), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "X:" + to_string((int)(tVec.at<double>(0))), Point(50, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Y:" + to_string((int)(tVec.at<double>(1))), Point(250, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Z:" + to_string((int)(tVec.at<double>(2))), Point(400, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        imshow("AngleSolver",angleImage);
    }
    if(showTVec)
    {
        cout << "tVec:" << endl;
        cout << " X:" << tVec.at<double>(0, 0);
        cout << " Y:" << tVec.at<double>(1, 0);
        cout << " Z:" << tVec.at<double>(2, 0);
        cout << endl;
        cout << "-----------------------------------------------" << endl;
    }
    float yaw_temp = x_yaw, pitch_temp = y_pitch;
    if(showP4P)
    {
        P4P_solver();
        cout<<"P4P Solver:"<<endl;
        cout<<"Yaw: " << x_yaw << "Pitch: " << y_pitch <<endl;
        cout << "-----------------------------------------------" << endl;
        x_yaw = yaw_temp; y_pitch = pitch_temp;
    }
    if(showPinHole)
    {
        PinHole_solver();
        cout<<"PinHole Solver:"<<endl;
        cout<<"Yaw: " << x_yaw << "Pitch: " << y_pitch <<endl;
        cout << "-----------------------------------------------" << endl;
        x_yaw = yaw_temp; y_pitch = pitch_temp;
    }
    if(showCompensation)
    {
        solveAngles();
        float raw_pitch;
        raw_pitch = y_pitch;

        projectileTransform(y_pitch);
        cout<<"Compensation:"<<endl;
        cout<<"Pitch Compensation:"<<y_pitch - raw_pitch<<endl;
        cout<<"Yaw: " << x_yaw << "Pitch: " << y_pitch <<endl;
        cout << "-----------------------------------------------" << endl;
        x_yaw = yaw_temp; y_pitch = pitch_temp;
    }
    if(showCameraParams)
    {
        cout<<"CANERA_MATRIX:"<<endl;
        cout<<CAMERA_MATRIX<<endl;
        cout << "-----------------------------------------------" << endl;
        cout<<"DISTORTION_COEFF"<<endl;
        cout<<DISTORTION_COEFF<<endl;
        cout << "-----------------------------------------------" << endl;
    }
}

