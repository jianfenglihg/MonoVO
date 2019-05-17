#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"common_include.h"

namespace myslam
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
    ~Mappoint();

    static Mappoint::Ptr createPoint();

};

} // myslam


#endif