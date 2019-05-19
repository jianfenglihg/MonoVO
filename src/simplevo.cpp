#include"simplevo.h"
#include<algorithm>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<boost/timer.hpp>

#include"config.h"

namespace slam
{

Simplevo::~Simplevo()
{}

Simplevo::Simplevo()
:vo_state_(UNINITIALIZED), ref_frame_(nullptr), cur_frame_(nullptr), num_lost_(0), num_inliers_(0)
{
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factors_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<float>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot_ = Config::get<double>("keyframe_rotation");
    key_frame_min_trans_ = Config::get<double>("keyframe_translation");
    //orb_ = cv::ORB::create(num_of_features_, scale_factors_, level_pyramid_);
    orb_ = cv::ORB::create();
}


bool Simplevo::addFrame(Frame::Ptr frame)
{
    switch (vo_state_)
    {
        case UNINITIALIZED:
        {
            cur_frame_  = frame;
            cur_frame_->R_ = ( cv::Mat_<double>(3,3)<< 
            1,0,0,
            0,1,0,
            0,0,1);
            cur_frame_->t_ = ( cv::Mat_<double>(3,1)<< 
            0,0,0);
            extractKeyPoints();
            computeDescriptors();
            ref_frame_ = cur_frame_;
            descriptor_curr_.copyTo(descriptor_ref_);
            key_point_ref_ = key_point_curr_;
            vo_state_ = INITIALIZING;
            break;
        }
        case INITIALIZING:
        {
            cur_frame_ = frame;
            //std::cout<<"insert key frame have done!"<<std::endl;
            extractKeyPoints();
            computeDescriptors();
            featureMatch();
            epipolorSolve();
            getAbsoluteScale(cur_frame_->id_);
            if(absolute_scale_ > 0.1)
            {
                cur_frame_->t_ = ref_frame_->t_ + absolute_scale_*ref_frame_->R_*relative_t_;
                cur_frame_->R_ = relative_R_*ref_frame_->R_;
            }
            else
            {
                cur_frame_->R_ = ref_frame_->R_;
                cur_frame_->t_ = ref_frame_->t_;
            }
            ref_frame_ = cur_frame_;
            descriptor_curr_.copyTo(descriptor_ref_);
            key_point_ref_ = key_point_curr_;
            num_lost_ = 0;
            vo_state_ = NORMAL;
            //std::cout<<"initializing have done!"<<std::endl;
            break;
        }
        case NORMAL:
        {
            cur_frame_ = frame;
            extractKeyPoints();
            computeDescriptors();
            featureMatch();
            epipolorSolve();
            getAbsoluteScale(cur_frame_->id_);
            /*
            if(checkEstimatedPose() == false)
            {
                num_lost_++;
                if(num_lost_>max_num_lost_) vo_state_ = LOST;
                return false;
            }
            */
            if(absolute_scale_ > 0.1)
            {
                cur_frame_->t_ = ref_frame_->t_ + absolute_scale_*ref_frame_->R_*relative_t_;
                cur_frame_->R_ = relative_R_*ref_frame_->R_;
            }
            else
            {
                cur_frame_->R_ = ref_frame_->R_;
                cur_frame_->t_ = ref_frame_->t_;
            }
            ref_frame_ = cur_frame_;
            descriptor_curr_.copyTo(descriptor_ref_);
            key_point_ref_ = key_point_curr_;
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

void Simplevo::extractKeyPoints()
{
    orb_->detect(cur_frame_->rgb_, key_point_curr_);
}

void Simplevo::computeDescriptors()
{
    orb_->compute(cur_frame_->rgb_,key_point_curr_,descriptor_curr_);
}


void Simplevo::featureMatch()
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
    //std::cout<< matches.size()<<std::endl;
    for ( cv::DMatch& m : matches )
    {
        //std::cout<< m.distance<<std::endl;

        if ( m.distance <= std::max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            matches_.push_back(m);
        }
    }
    //std::cout<<"good matches: "<<matches_.size()<<std::endl;
}


void Simplevo::epipolorSolve()
{
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for ( int i = 0; i < ( int ) matches_.size(); i++ )
    {
        //std::cout<< key_point_curr_[matches_[i].queryIdx].pt<< key_point_ref_[matches_[i].trainIdx].pt<<std::endl;
        points1.push_back ( key_point_curr_[matches_[i].trainIdx].pt );
        points2.push_back ( key_point_ref_[matches_[i].queryIdx].pt );
    }
  
    cv::Point2d principal_point ( cur_frame_->cam_->cx_, cur_frame_->cam_->cy_ );	
    double focal_length = cur_frame_->cam_->fx_;
    Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point );
    
    Mat R,t;
    cv::recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    R.copyTo(relative_R_);
    t.copyTo(relative_t_);
}



 bool Simplevo::checkEstimatedPose()
 {
    double distance = sqrt(relative_t_.at<float>(0,0)*relative_t_.at<float>(0,0)+
    relative_t_.at<float>(1,0)*relative_t_.at<float>(1,0)+
    relative_t_.at<float>(2,0)*relative_t_.at<float>(2,0));
    if ( distance > 5.0 )
    {
        std::cout<<"reject because motion is too large: "<<distance<<std::endl;
        return false;
    }
    return true;
 }


int Simplevo::hanmingDistance(Mat str1, Mat str2) 
{
    if (str1.cols != str2.cols)
        return -1;
    int difference = 0;
    for (int i = 0; i < str1.cols; i++) {
        if (str1.at<uchar>(0,i) != str2.at<uchar>(0,i))
            difference++;
    }
    return difference;
}

void Simplevo::getAbsoluteScale(long frame_id)
{
    absolute_scale_ = 1.0;
}

}

