#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"common_include.h"

namespace slam
{
class Mappoint
{
private:
    /* data */
public:
    typedef std::shared_ptr<Mappoint> Ptr;
    unsigned long id_;
    Vector3d point_pos_;
    Vector3d view_direction_;
    int observed_times_;
    int correct_times_;
    Mat descriptor_;
public:
    Mappoint();
    Mappoint(long id, Vector3d point_pos, Vector3d view_direction);
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( point_pos_(0,0), point_pos_(1,0), point_pos_(2,0) );
    }
    ~Mappoint();

    static Mappoint::Ptr createPoint();
    static Mappoint::Ptr createPoint( Vector3d point_poss);

};

} // myslam


#endif