#include"frame.h"

namespace myslam
{

Frame::Frame()
:id_(-1), time_stamp_(-1), cam_(nullptr)
{}

Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr cam, Mat color, Mat depth)
:id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), cam_(cam), rgb_(color), depth_(depth)
{}

Frame::~Frame()
{}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}  

double Frame::findDepth(const cv::KeyPoint& pt_world)
{
    int x = cvRound(pt_world.pt.x);
    int y = cvRound(pt_world.pt.y);

    ushort d = depth_.ptr<ushort>(y)[x];
    if(d!=0)
    {
        return double(d)/cam_->depth_scale_;
    }
    else
    {
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/cam_->depth_scale_;
            }
        }
    }
    return -1.0;
}

bool Frame::isInFrame(const Vector3d& pt_world)
{
    Vector3d pt_cam = cam_->world2cam(T_c_w_,pt_world);
    if(pt_cam(2,0)<0) return false;

    Vector2d pt_pixel = cam_->cam2pixel(pt_cam);

    return pt_pixel(0,0)>0 && pt_pixel(1,0)>0 
        && pt_pixel(0,0)<rgb_.cols 
        && pt_pixel(1,0)<rgb_.rows;
}

Vector3d Frame::getCameraCenter() const
{
    return T_c_w_.inverse().translation();
}


} 

