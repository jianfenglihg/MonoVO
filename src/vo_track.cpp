#include"vo_track.h"


namespace slam
{

Trackvo::~Trackvo()
{}

Trackvo::Trackvo()
:vo_state_(INITIALIZING), ref_frame_(nullptr), cur_frame_(nullptr)
{
}


bool Trackvo::addFrame(Frame::Ptr frame)
{
    switch (vo_state_)
    {
        case INITIALIZING:
        {
            cur_frame_  = frame;
            cur_frame_->R_ = ( cv::Mat_<double>(3,3)<< 
            1,0,0,
            0,1,0,
            0,0,1);
            cur_frame_->t_ = ( cv::Mat_<double>(3,1)<< 
            0,0,0);

            ref_frame_ = cur_frame_;
            featureDetection();
            vo_state_ = NORMAL;
            break;
        }
        case NORMAL:
        {
            cur_frame_ = frame;
            //std::cout<<"insert key frame have done!"<<std::endl;
            featureTracking();
            if (key_point_curr_.size() <  Config::get<int>("number_of_features"))	
            {
                featureDetection();
                featureTracking();
            }
            epipolorSolve();
            getAbsoluteScale(cur_frame_->id_);
            //std::cout<<"scale"<<absolute_scale_<<std::endl;
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
            key_point_ref_ = key_point_curr_;

            vo_state_ = NORMAL;
            //std::cout<<"initializing have done!"<<std::endl;
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


void Trackvo::epipolorSolve()
{
    cv::Point2d principal_point ( cur_frame_->cam_->cx_, cur_frame_->cam_->cy_ );	
    double focal_length = cur_frame_->cam_->fx_;
    Mat essential_matrix;
    essential_matrix = cv::findEssentialMat ( key_point_curr_, key_point_ref_, focal_length, principal_point );
    
    Mat R,t;
    cv::recoverPose ( essential_matrix, key_point_curr_, key_point_ref_, R, t, focal_length, principal_point );
    relative_R_ = R.clone();
    relative_t_ = t.clone();
}

void Trackvo::featureTracking()
{
    //this function automatically gets rid of points for which tracking fails

    std::vector<float> err;	
    std::vector<uchar> status;			
    Size winSize=Size(21,21);																								
    cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(ref_frame_->rgb_, cur_frame_->rgb_, key_point_ref_, key_point_curr_, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  
        cv::Point2f pt = key_point_curr_.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	
        {
            if((pt.x<0)||(pt.y<0))	
            {
                status.at(i) = 0;
            }
            key_point_ref_.erase (key_point_ref_.begin() + (i - indexCorrection));
            key_point_curr_.erase (key_point_curr_.begin() + (i - indexCorrection));
            indexCorrection++;
        }

    }

}


void Trackvo::featureDetection()
{
    std::vector<cv::KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(ref_frame_->rgb_, keypoints_1, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints_1, key_point_ref_, std::vector<int>());
}

void Trackvo::getAbsoluteScale(long frame_id)
{
    std::string line;
    int i = 0;
    std::ifstream myfile ("/home/ljf/code/00.txt");
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
            in >> z ;
            if (j==7) y=z;
            if (j==3)  x=z;
            }
            
            i++;
        }
        myfile.close();
    }

    else {
    std::cout << "Unable to open file";
    }

    absolute_scale_ = sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
}

}

