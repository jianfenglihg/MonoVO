#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H
#include<iostream>
#include<memory>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include"sophus/so3.h"
#include"sophus/se3.h"

using Sophus::SE3;
using Sophus::SO3;
using cv::Mat;
using Eigen::Vector3d;
using Eigen::Vector2d;

#endif