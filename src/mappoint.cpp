#include"mappoint.h"

namespace slam
{
Mappoint::Mappoint()
:id_(-1), point_pos_(Vector3d(0,0,0)), view_direction_(Vector3d(0,0,0)),observed_times_(0), correct_times_(0)
{

}

Mappoint::~Mappoint()
{}

Mappoint::Mappoint(long id, Vector3d point_pos, Vector3d view_direction)
:id_(id), point_pos_(point_pos), view_direction_(view_direction),observed_times_(0), correct_times_(0)
{

}

Mappoint::Ptr Mappoint::createPoint()
{
    static long point_id = 0;
    return(Mappoint::Ptr(new Mappoint(point_id++, Vector3d(0,0,0), Vector3d(0,0,0))));
}

Mappoint::Ptr Mappoint::createPoint( Vector3d point_pos)
{
    static long point_id = 0;
    return(Mappoint::Ptr(new Mappoint(point_id++, point_pos, Vector3d(0,0,0))));
}

} // slam
