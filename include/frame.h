#ifndef FRAME_H
#define FRAME_H

#include"common_include.h"
#include"camera.h"

namespace slam
{

class MapPoint;
class Frame
{
private:
    /* data */
public:
    typedef std::shared_ptr<Frame> Ptr;
    long id_;
    cv::Mat rgb_, depth_;
    Camera::Ptr cam_;
    SE3 T_c_w_;
    cv::Mat Tcw;
    double time_stamp_;

public:
    Frame();
    Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr cam=nullptr, Mat color=Mat(), Mat depth=Mat());
    ~Frame();
    static Frame::Ptr createFrame();
    double findDepth(const cv::KeyPoint& kp);
    Vector3d getCameraCenter() const;
    bool isInFrame(const Vector3d& pt_world);
    void se3ToT34();
};
   
}

#endif