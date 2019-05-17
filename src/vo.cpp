#include"vo.h"
#include<algorithm>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<boost/timer.hpp>

#include"config.h"

namespace myslam
{

Vo::~Vo()
{}

Vo::Vo()
:vo_state_(INITIALIZING), ref_frame_(nullptr), cur_frame_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
{
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factors_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<float>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot_ = Config::get<double>("keyframe_rotation");
    key_frame_min_trans_ = Config::get<double>("keyframe_translation");
    orb_ = cv::ORB::create(num_of_features_, scale_factors_, level_pyramid_);
    //orb_ = cv::ORB::create();
}


bool Vo::addFrame(Frame::Ptr frame)
{
    switch (vo_state_)
    {
        case INITIALIZING:
        {
            cur_frame_ = ref_frame_ = frame;
            map_->insertKeyFrame(frame);
            //std::cout<<"insert key frame have done!"<<std::endl;
            extractKeyPoints();
            //std::cout<<"extract key point have done!"<<std::endl;
            computeDescriptors();
            setRefPoint3d();
            vo_state_ = NORMAL;
            break;
        }
        case NORMAL:
        {
            cur_frame_ = frame;
            extractKeyPoints();
            computeDescriptors();
            featureMatch();
            poseEstimatePnP();
            if(checkEstimatedPose() == true)
            {
                cur_frame_->T_c_w_ = T_c_r_*ref_frame_->T_c_w_;
                ref_frame_ = cur_frame_;
                setRefPoint3d();
                num_lost_ = 0;
                if(checkKeyFrame() == true) addKeyFrame();
            }
            else
            {
                num_lost_++;
                if(num_lost_>max_num_lost_) vo_state_ = LOST;
                return false;
            }
            break;
        }
        case LOST:
        {
            std::cout<<"vo has lost."<<std::endl;
            break;
        }
    }
    return true;
}

void Vo::extractKeyPoints()
{
    orb_->detect(cur_frame_->rgb_, key_point_curr_);
}

void Vo::computeDescriptors()
{
    orb_->compute(cur_frame_->rgb_,key_point_curr_,descriptor_curr_);
}

void Vo::featureMatch()
{
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptor_ref_,descriptor_curr_,matches);
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance <= std::max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            matches_.push_back(m);
        }
    }
    std::cout<<"good matches: "<<matches_.size()<<std::endl;
}

void Vo::setRefPoint3d()
{
    points_ref_3d_.clear();
    descriptor_ref_ = Mat();
    
    for ( size_t i=0; i<key_point_curr_.size(); i++ )
    {
        double d = ref_frame_->findDepth(key_point_curr_[i]);               
        if ( d > 0)
        {
            Vector3d p_cam = ref_frame_->cam_->pixel2cam(
                Vector2d(key_point_curr_[i].pt.x, key_point_curr_[i].pt.y), d
            );
            points_ref_3d_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptor_ref_.push_back(descriptor_curr_.row(i));
        }
    }
}

void Vo::poseEstimatePnP()
{
    std::vector<cv::Point3f> point3d;
    std::vector<cv::Point2f> point2d;
    for(cv::DMatch m : matches_)
    {
        point3d.push_back(points_ref_3d_[m.queryIdx]);
        point2d.push_back(key_point_curr_[m.trainIdx].pt);
    }
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_frame_->cam_->fx_, 0, ref_frame_->cam_->cx_,
        0, ref_frame_->cam_->fy_, ref_frame_->cam_->cy_,
        0,0,1
    );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(point3d,point2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers);
    num_inliers_ = inliers.rows;
    std::cout<<"pnp inliers: "<<num_inliers_<<std::endl;
    T_c_r_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}
 bool Vo::checkEstimatedPose()
 {
     if ( num_inliers_ < min_inliers_ )
    {
        std::cout<<"reject because inlier is too small: "<<num_inliers_<<std::endl;
        return false;
    }
    Sophus::Vector6d d = T_c_r_.log();
    if ( d.norm() > 5.0 )
    {
        std::cout<<"reject because motion is too large: "<<d.norm()<<std::endl;
        return false;
    }
    return true;
 }

bool Vo::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot_ || trans.norm() >key_frame_min_trans_ )
        return true;
    return false;
}

void Vo::addKeyFrame()
{
    std::cout<<"adding a key-frame"<<std::endl;
    map_->insertKeyFrame(cur_frame_);
}

}
