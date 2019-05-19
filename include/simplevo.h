#ifndef SIMPLEVO_H
#define SIMPLEVO_H

#include"common_include.h"
#include"frame.h"
#include <string>

namespace slam
{

class Simplevo
{
public:
typedef std::shared_ptr<Simplevo> Ptr;
enum VoState {
    UNINITIALIZED=-2,
    INITIALIZING=-1,
    LOST=0,
    NORMAL=1
};
VoState vo_state_;
Frame::Ptr ref_frame_;
Frame::Ptr cur_frame_;

cv::Ptr<cv::ORB> orb_;

std::vector<cv::KeyPoint> key_point_curr_;
std::vector<cv::KeyPoint> key_point_ref_;
cv::Mat descriptor_curr_;
cv::Mat descriptor_ref_;

std::vector<cv::DMatch> matches_;

SE3 T_c_r_;
cv::Mat relative_R_;
cv::Mat relative_t_;

int num_inliers_;
int num_lost_;
double absolute_scale_;

int num_of_features_;
double scale_factors_;
int level_pyramid_;
float match_ratio_;
int max_num_lost_;
int min_inliers_;

double key_frame_min_rot_;
double key_frame_min_trans_;

public:
    Simplevo(/* args */);
    ~Simplevo();

    bool addFrame(Frame::Ptr frame);

public:
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatch();
    void epipolorSolve();
    void getAbsoluteScale(long frame_id);
    
    bool checkEstimatedPose();


    int hanmingDistance(Mat str1, Mat str2);

};


} // myslam



#endif