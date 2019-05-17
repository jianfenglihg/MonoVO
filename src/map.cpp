#include"myslam/map.h"
namespace myslam
{
Map::Map()
{}
Map::~Map()
{}

void Map::insertKeyFrame(Frame::Ptr key_frame)
{
    std::cout<<"key-frames's size:"<<key_frames_.size()<<std::endl;
    if (key_frames_.find(key_frame->id_)==key_frames_.end()) {
        key_frames_.insert(std::make_pair(key_frame->id_, key_frame));
    }
    else
    {
        key_frames_[key_frame->id_] = key_frame; 
    }
    
    
}

void Map::insertPoint(Mappoint::Ptr map_point)
{
    std::cout<<"map-point's size:"<<map_points_.size()<<std::endl;
    if (map_points_.find(map_point->id_)==map_points_.end()) {
        map_points_.insert(std::make_pair(map_point->id_, map_point));
    }
    else
    {
        map_points_[map_point->id_] = map_point;
    }
    
}
} // myslam
