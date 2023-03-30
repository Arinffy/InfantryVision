#pragma once
/*
*	@Author: USTL-COD
*	@Date:	 2022.04.04
*	@Brief:  This header file declares all the classes and params used to solve eular angle
*/
#include "main.h"
#include "ArmorDetector.h"
using namespace cv;
using namespace std;

class AngleSolver
{

public :
    AngleSolver();
    ~AngleSolver();

    void updateKFParam(const double& delt_time);

    /*
    * @brief Set camera params
    * @param camera_matrix: camera IntrinsicMatrix
    * @param distortion_coeff: camera DistortionCoefficients
    * overload function. Params set by xml file
    */
    int setCameraParam(const char* filePath, int camId);

    /*
    * @brief Set armor size
    * @param type: input target type small/big
    * @param width: the width of armor (mm)
    * @param height: the height of armor (mm)
    */
    void setArmorSize(ArmorType type, double width, double height);

    /*
    * @brief Set bullet speed
    * @param bulletSpeed: the speed of bullet(mm/s)
    */
    void setBulletSpeed(int bulletSpeed);
    void setGyroYaw(float yaw);
    void setGyroPitch(float pitch);

    /*
    * @brief set the target armor contour points and centerPoint
    * @param points2d image points set with following order : left_up, right_up, left_down, right_down
    * @param type target armor type
    */
    void setTarget(vector<Point2f> contoursPoints, Point2f centerPoint, ArmorType type);


    /*
    * @brief solve the angles using P4P or PinHole according to the distance
    */
    void solveAngles();

    /*
    * @brief solve the angles using P4P method
    */
    void P4P_solver();

    /*
    * @brief solve the angles using PinHole method
    */
    void PinHole_solver();

    Vec4f getAngularv(const double& now_delt_time);
    void anglePredict();
    void Spinobserver();
    /*
    * @brief compensation of pitch
    */
    void projectileInit(float x,float y,float z,float pitch,float yaw);

    /*
    * @brief compensation of pitch for y_offset between barrel and camera
    */
    float projectileModel(float tardis,float bollv,float pitch);

    /*
    * @brief compensation of pitch for gravity
    */
    float projectileGet(float tarx,float tary,float bollv);

    void projectileTransform(float& pitch);

    /*
    * @brief according to the target2D points to get the yaw and pitch and distance towards the certain type target using solvePnP
    * @param inputArrayOfPoints contourPoints, the vertices of target armor
    * @param inputPoint centerPoint the center point of target armor
    * @param input type the type of armor BIG_ARMOR or SMALL_ARMOR
    * @param output y_yaw angle     the angle that yaw revolve     '<-' left is minus-       '->' right is positive+
    * @param output x_pitch angle   the angle that pitch revolve   '下' down is minus-       '上' up is positive+
    * @param output distance  unit is mm
    */
    void getAngle(vector<Point2f>& contourPoints, Point2f centerPoint, ArmorType type, double & yaw, double & pitch, double & evaluateDistance,Point2f& precenter);

    /*
    * @brief show debug information
    */
    void showDebugInfo(bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams);

private:

    //Camera params
    Mat CAMERA_MATRIX;    //IntrinsicMatrix		  fx,fy,cx,cy
    Mat DISTORTION_COEFF; //DistortionCoefficients k1,k2,p1,p2

    //Object points in world coordinate
    vector<Point3f> SMALL_ARMOR_POINTS_3D;
    vector<Point3f> BIG_ARMOR_POINTS_3D;

    //Targets
    vector<Point2f> targetContour;
    Point2f targetCenter;
    ArmorType targetType;

    // calculated by solvePnP
    //s[R|t]=s'  s->world coordinate;s`->camera coordinate
    Mat rVec;    //rot rotation between camera and target center
    Mat tVec;  //trans tanslation between camera and target center

    double x_pos;
    double y_pos;
    double z_pos;

    float bullet_speed;
    float gyro_pitch;
    float gyro_yaw;

    //projectile

    bool isNumdif;
    enum State{
        NO_FOUND,
        DETECTING,
        TRACKING,
        LOST,
    }track_state;

    float offset_x;//compensation of x
    float offset_y;//compensation of y
    float offset_z;//compensation of z
    float offset_pitch;//compensation of pitch
    float offset_yaw;//compensation of yaw
    float friction_factor;//0.47
    float friction_density;//0-1.293 25-1.169 kg/m^3
    float friction_radius;
    float friction_weight;
    float friction_contact;//All:small- big-  Right:small- big-
    float friction_cure;//the end factor
    float gravity;
    float flytime;

    //Results
    float x_yaw;
    float y_pitch;
    double distance;

    //predict
    Mat measurement;
    Mat controlment;
    Point2f tarpredict;
    Mat predictmatrix;
    Point2f predictcenter;
    Point2f predictangle;
    Point2f predictangularv;
    Vec4f evaluateact;

    Point2f now_angular_vel;
    Point2f now_angular_acc;
    //spinobserver
    double max_jump_angle;
    double max_jump_period;
    double allow_following_range;
    bool target_spinning_;

    double jump_period_;
    int jump_count_;

    double last_yaw_;
    double last_jump_yaw_diff_;
    double last_jump_time_;
    Point2f last_jump_position_;
};


