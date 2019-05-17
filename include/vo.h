#ifndef VO_H
#define VO_H

#include"common_include.h"
#include"map.h"

namespace myslam
{
class Vo
{
private:
    /* data */
public:
typedef std::shared_ptr<Vo> Ptr;
enum VoState {
    INITIALIZING=-1,
    LOST=0,
    NORMAL=1
};
VoState vo_state_;
Map::Ptr map_;
Frame::Ptr ref_frame_;
Frame::Ptr cur_frame_;
cv::Ptr<cv::ORB> orb_;
std::vector<cv::Point3f> points_ref_3d_;
std::vector<cv::KeyPoint> key_point_curr_;
cv::Mat descriptor_curr_;
cv::Mat descriptor_ref_;
std::vector<cv::DMatch> matches_;
SE3 T_c_r_;
int num_inliers_;
int num_lost_;

int num_of_features_;
double scale_factors_;
int level_pyramid_;
float match_ratio_;
int max_num_lost_;
int min_inliers_;

double key_frame_min_rot_;
double key_frame_min_trans_;

public:
    Vo(/* args */);
    ~Vo();

    bool addFrame(Frame::Ptr frame);

public:
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatch();
    void poseEstimatePnP();
    void setRefPoint3d();
    void addKeyFrame();

bool checkEstimatedPose();
bool checkKeyFrame();

};


} // myslam



#endif