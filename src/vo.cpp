#include"vo.h"
#include<algorithm>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<boost/timer.hpp>

#include"config.h"

namespace slam
{

Vo::~Vo()
{}

Vo::Vo()
:vo_state_(UNINITIALIZED), ref_frame_(nullptr), cur_frame_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0), _local_map(new Localmap(Config::get<int>("local_map_size")))
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
}


bool Vo::addFrame(Frame::Ptr frame)
{
    switch (vo_state_)
    {
        case UNINITIALIZED:
        {
            cur_frame_  = frame;
            cur_frame_->T_c_w_ = SE3(SO3(0.0, 0.0, 0.0), Vector3d( 0.0, 0.0, 0.0));
            cur_frame_->Tcw = ( cv::Mat_<double>(3,4)<< 
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);
            map_->key_frames_.push_back(cur_frame_);
            _local_map->addKeyFrame(cur_frame_);
            extractKeyPoints();
            computeDescriptors();
            ref_frame_ = cur_frame_;
            descriptor_ref_ = descriptor_curr_;
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
            cur_frame_->T_c_w_ = T_c_r_*ref_frame_->T_c_w_;
            ref_frame_ = cur_frame_;
            num_lost_ = 0;
            map_->key_frames_.push_back(cur_frame_);
            _local_map->addKeyFrame(cur_frame_);
            //std::cout<<"update map begin"<<std::endl;
            updateMap();
            //std::cout<<"update map end"<<std::endl;
            vo_state_ = NORMAL;
            //std::cout<<"initializing have done!"<<std::endl;
            break;
        }
        case NORMAL:
        {
            cur_frame_ = frame;
            extractKeyPoints();
            computeDescriptors();
            featureMatchFromMap();
            poseEstimatePnP();
            if(checkEstimatedPose() == true)
            {
                cur_frame_->T_c_w_ = T_c_map_;
                ref_frame_ = cur_frame_;
                if(checkKeyFrame() == true) 
                {
                    addKeyFrame();
                    updateMap();
                    _local_map->addKeyFrame(cur_frame_);
                }
            }
            else
            {
                num_lost_++;
                if(num_lost_>max_num_lost_) vo_state_ = LOST;
                return false;
            }
            //std::cout<<"Normal have done!"<<std::endl;
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

void Vo::featureMatchFromMap()
{
    std::vector<cv::DMatch> matches;
    //generate candinate points3d from map;
    for(auto iter : _local_map->map_points_)
    {
        if(cur_frame_->isInFrame(iter->point_pos_))
        {
            iter->observed_times_++;
            pt3d_candinate_.push_back(iter);
            descriptor_candinate_.push_back(iter->descriptor_);
        }
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptor_candinate_,descriptor_curr_,matches);
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    matches_map_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance <= std::max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            matches_map_.push_back(m);
        }
    }
    std::cout<< "matches from map number: "<<matches_map_.size()<<std::endl;
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
    //std::cout<<"good matches: "<<matches_.size()<<std::endl;
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

void Vo::epipolorSolve()
{
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for ( int i = 0; i < ( int ) matches_.size(); i++ )
    {
        points1.push_back ( key_point_curr_[matches_[i].queryIdx].pt );
        points2.push_back ( key_point_ref_[matches_[i].trainIdx].pt );
    }
    Mat fundmental_matrix;
    fundmental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);

    
    cv::Point2d principal_point ( cur_frame_->cam_->cx_, cur_frame_->cam_->cy_ );	
    double focal_length = cur_frame_->cam_->fx_;
    Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( points1, points2, focal_length, principal_point );
    
    Mat R,rvec,t;
    cv::recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cv::Rodrigues(R,rvec);
    T_c_r_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))
    );
    cur_frame_->Tcw = ( cv::Mat_<double>(3,4)<<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),t.at<double>(2,0)
    );
    //std::cout<< "epipolar calculate completed" << std::endl;

}

void Vo::poseEstimatePnP()
{
    std::vector<cv::Point3f> point3d;
    std::vector<cv::Point2f> point2d;
    for(cv::DMatch m : matches_map_)
    {
        point3d.push_back(pt3d_candinate_[m.queryIdx]->getPositionCV());
        point2d.push_back(key_point_curr_[m.trainIdx].pt);
    }
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_frame_->cam_->fx_, 0, ref_frame_->cam_->cx_,
        0, ref_frame_->cam_->fy_, ref_frame_->cam_->cy_,
        0,0,1
    );
    Mat rvec, tvec, inliers, RR;
    cv::solvePnPRansac(point3d,point2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers);
    num_inliers_ = inliers.rows;
    std::cout<<"pnp inliers: "<<num_inliers_<<std::endl;
    T_c_map_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    cv::Rodrigues(rvec,RR);
    cur_frame_->Tcw = ( cv::Mat_<double>(3,4)<<
        RR.at<double>(0,0), RR.at<double>(0,1), RR.at<double>(0,2),tvec.at<double>(0,0),
        RR.at<double>(1,0), RR.at<double>(1,1), RR.at<double>(1,2),tvec.at<double>(1,0),
        RR.at<double>(2,0), RR.at<double>(2,1), RR.at<double>(2,2),tvec.at<double>(2,0)
    );
}

void Vo::updateMap()
{
    std::cout<<"the size of key frame:  "<< map_->key_frames_.size()<<std::endl;
    std::cout<<"the size of map point:  "<< map_->map_points_.size()<<std::endl;
    std::vector<Frame::Ptr>::iterator iter;
    iter = map_->key_frames_.end()-1;
    Frame::Ptr key_frame_cur, key_frame_ref;
    key_frame_cur = *iter;
    //std::cout<<"get key frame 1 have done!"<<std::endl;
    iter = iter -1 ;
    key_frame_ref = *iter;
    //std::cout<<"get key frame have done!"<<std::endl;

    Mat R,t;

    std::vector<cv::KeyPoint> key_frame_point_curr_;
    std::vector<cv::KeyPoint> key_frame_point_ref_;
    cv::Mat descriptor_frame_curr_;
    cv::Mat descriptor_frame_ref_;
    orb_->detect(key_frame_cur->rgb_, key_frame_point_curr_);
    orb_->detect(key_frame_ref->rgb_, key_frame_point_ref_);
    orb_->compute(key_frame_cur->rgb_,key_frame_point_curr_,descriptor_frame_curr_);
    orb_->compute(key_frame_ref->rgb_,key_frame_point_ref_,descriptor_frame_ref_);
    //std::cout<<"fp detect have done!"<<std::endl;

    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptor_frame_ref_,descriptor_frame_curr_,matches);
    //std::cout<<"match_1 have done!"<<std::endl;
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;
    //std::cout<<"match_2 have done!"<<std::endl;
    std::vector<cv::DMatch>::iterator iter_match;
    for ( iter_match=matches.begin(); iter_match != matches.end(); iter_match++  )
    {
        if ( iter_match->distance > std::max<float> ( min_dis*match_ratio_, 200.0 ) )
        {
            matches.erase(iter_match);
        }
    }
    //std::cout<<"match have done!"<<std::endl;

    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_frame_->cam_->fx_, 0, ref_frame_->cam_->cx_,
        0, ref_frame_->cam_->fy_, ref_frame_->cam_->cy_,
        0,0,1
    );
    std::vector<cv::Point2f> pts_1, pts_2;
    Mat desc;
    std::vector<int> d;
    int dmin = 8000;
    for ( auto m:matches )
    {
        if(vo_state_ == NORMAL)
        {
            for( auto piter : _local_map->map_points_)
            {
                if(piter->_frame_id != key_frame_ref->id_ && piter->_frame_id != key_frame_cur->id_ )
                {
                    int di = hanmingDistance(piter->descriptor_.row(0),descriptor_frame_curr_.row(m.trainIdx));
                    if(dmin > di) dmin = di;
                }
            }
            if(dmin>Config::get<float>("match_ratio_inmap"))
            {
                pts_1.push_back ( key_frame_ref->cam_->pixel2cam_cv( key_frame_point_ref_[m.queryIdx].pt, K) );
                pts_2.push_back ( key_frame_cur->cam_->pixel2cam_cv( key_frame_point_curr_[m.trainIdx].pt, K) );
                desc.push_back(descriptor_frame_curr_.row(m.trainIdx));
            }
        }
        else
        {
            pts_1.push_back ( key_frame_ref->cam_->pixel2cam_cv( key_frame_point_ref_[m.queryIdx].pt, K) );
            pts_2.push_back ( key_frame_cur->cam_->pixel2cam_cv( key_frame_point_curr_[m.trainIdx].pt, K) );
            desc.push_back(descriptor_frame_curr_.row(m.trainIdx));
        }
    }
    std::cout<< "new points number: "<<pts_1.size()<<std::endl;
    Mat pts_4d;
    if(pts_1.size() != 0)
    {
        cv::triangulatePoints( key_frame_ref->Tcw, key_frame_cur->Tcw, pts_1, pts_2, pts_4d );

        //std::cout<<"triangular have done!"<<std::endl;
        
        for ( int i=0; i<pts_4d.cols; i++ )
        {
            Mat x = pts_4d.col(i);
            x /= x.at<float>(3,0);
            Vector3d point(
                x.at<float>(0,0), 
                x.at<float>(1,0), 
                x.at<float>(2,0) 
            );
            Mappoint::Ptr p = Mappoint::createPoint(point, key_frame_cur->id_);
            p->descriptor_.push_back(desc.row(i));
            map_->map_points_.push_back(p); 
            _local_map->map_points_.push_back(p);    
        }
    }
    std::cout<<"local map point number: "<< _local_map->map_points_.size()<<std::endl;
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
    //std::cout<<"adding a key-frame"<<std::endl;
    map_->key_frames_.push_back(cur_frame_);
}

int Vo::hanmingDistance(Mat str1, Mat str2) 
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

}

