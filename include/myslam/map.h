#ifndef MAP_H
#define MAP_H

#include"common_include.h"
#include"frame.h"
#include"mappoint.h"

#include<unordered_map>

namespace myslam
{
class Map
{
private:
    /* data */
public:
typedef std::shared_ptr<Map> Ptr;
std::unordered_map<unsigned long, Frame::Ptr> key_frames_;
std::unordered_map<unsigned long, Mappoint::Ptr> map_points_;

public:
    Map(/* args */);
    ~Map();
    void insertPoint(Mappoint::Ptr map_point);
    void insertKeyFrame(Frame::Ptr key_frame);
};

} // myslam


#endif