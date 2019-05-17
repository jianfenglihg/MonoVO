#ifndef VO_H
#define VO_H

#include"common_include.h"
#include"map.h"

namespace slam
{
class Vo
{
private:
    /* data */
public:
typedef std::shared_ptr<Vo> Ptr;
enum VoState {
    UNINITIALIZED=-2,
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
std::vector<cv::KeyPoint> key_point_ref_;
cv::Mat descriptor_curr_;
cv::Mat descriptor_ref_;
std::vector<Mappoint::Ptr> pt3d_candinate_;
Mat descriptor_candinate_;
std::vector<cv::DMatch> matches_;
std::vector<cv::DMatch> matches_map_;
SE3 T_c_r_;
SE3 T_c_map_;
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
    void featureMatchFromMap();
    void poseEstimatePnP();
    void setRefPoint3d();
    void addKeyFrame();
    void updateMap();
    void epipolorSolve();

bool checkEstimatedPose();
bool checkKeyFrame();

};


} // myslam



#endif