#ifndef VO_TRACK_H
#define VO_TRACK_H

#include"common_include.h"
#include"frame.h"
#include <string>
#include <sstream>
#include <fstream>
#include<algorithm>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/video/tracking.hpp>
#include<boost/timer.hpp>

#include"config.h"

namespace slam
{

class Trackvo
{
public:
typedef std::shared_ptr<Trackvo> Ptr;
typedef cv::Size2i Size;
enum VoState {
    INITIALIZING=-1,
    LOST=0,
    NORMAL=1
};
VoState vo_state_;
Frame::Ptr ref_frame_;
Frame::Ptr cur_frame_;

std::vector<cv::Point2f> key_point_curr_;
std::vector<cv::Point2f> key_point_ref_;

cv::Mat relative_R_;
cv::Mat relative_t_;

double absolute_scale_;

public:
    Trackvo(/* args */);
    ~Trackvo();

    bool addFrame(Frame::Ptr frame);

public:
    void epipolorSolve();
    void getAbsoluteScale(long frame_id);

    void featureDetection();
    void featureTracking();
};


} // myslam



#endif