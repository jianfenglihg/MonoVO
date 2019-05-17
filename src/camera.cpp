#include"camera.h"
#include"common_include.h"
#include"config.h"

namespace myslam
{

Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}

Camera::~Camera()
{}

Vector3d Camera::pixel2cam(const Vector2d& point_image, double depth)
{
    return Vector3d(
        (point_image(0,0)*depth-cx_)/fx_,
        (point_image(1,0)*depth-cy_)/fy_,
        depth);
}
Vector3d Camera::cam2world(const SE3& T_c_w, const Vector3d& point_camera)
{
    return T_c_w.inverse()*point_camera;
}
Vector3d Camera::world2cam(const SE3& T_c_w, const Vector3d& point_world)
{
    return T_c_w*point_world;
}
Vector2d Camera::cam2pixel(const Vector3d& point_camera)
{
    return Vector2d(
        fx_*point_camera(0,0)/point_camera(2,0)+cx_, 
        fy_*point_camera(1,0)/point_camera(2,0)+cy_);
}

Vector2d Camera::world2pixel(const SE3& T_c_w, const Vector3d& point_world)
{
    return cam2pixel(world2cam(T_c_w, point_world));
}
Vector3d Camera::pixel2world(const SE3& T_c_w, const Vector2d& point_image, double depth)
{
    return cam2world(T_c_w, pixel2cam(point_image, depth));
}
}
