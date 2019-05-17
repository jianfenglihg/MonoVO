#ifndef CAMERA_H
#define CAMERA_H
#include"common_include.h"

namespace myslam
{
class Camera
{
private:
    /* data */
public:
    typedef std::shared_ptr<Camera> Ptr;
    float fx_, fy_, cx_, cy_, s_, depth_scale_;

    Camera();
    Camera(float fx, float fy, float cx, float cy, float s = 0, float depth_scale = 1):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), s_(s), depth_scale_(depth_scale)
    {}
    ~Camera();

    cv::Point2f Camera::pixel2cam_cv( const cv::Point2d& p, const cv::Mat& K );
    
    Vector3d world2cam(const SE3& T_c_w, const Vector3d& point_world);
    Vector2d cam2pixel(const Vector3d& point_camera);
    Vector3d pixel2cam(const Vector2d& point_image, double depth = 1);
    Vector3d cam2world(const SE3& T_c_w, const Vector3d& point_camera);

    Vector2d world2pixel(const SE3& T_c_w, const Vector3d& point_world);
    Vector3d pixel2world(const SE3& T_c_w, const Vector2d& point_image, double depth =1 );

};

}
#endif