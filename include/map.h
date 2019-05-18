#ifndef MAP_H
#define MAP_H

#include"common_include.h"
#include"frame.h"
#include"mappoint.h"

#include<unordered_map>

namespace slam
{
class Map
{
private:
    /* data */
public:
typedef std::shared_ptr<Map> Ptr;

std::vector<Frame::Ptr> key_frames_;
std::vector<Mappoint::Ptr> map_points_;

std::unordered_map<unsigned long, Frame::Ptr> _key_frames;
std::unordered_map<unsigned long, Mappoint::Ptr> _map_points;

public:
    Map();
    ~Map();
    void insertPoint(Mappoint::Ptr map_point);
    void insertKeyFrame(Frame::Ptr key_frame);
};

} // myslam


#endif