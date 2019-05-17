#include"map.h"
namespace slam
{
Map::Map()
{}
Map::~Map()
{}

void Map::insertKeyFrame(Frame::Ptr key_frame)
{
    std::cout<<"key-frames's size:"<<_key_frames.size()<<std::endl;
    if (_key_frames.find(key_frame->id_)==_key_frames.end()) {
        _key_frames.insert(std::make_pair(key_frame->id_, key_frame));
    }
    else
    {
        _key_frames[key_frame->id_] = key_frame; 
    }
    
    
}

void Map::insertPoint(Mappoint::Ptr map_point)
{
    std::cout<<"map-point's size:"<<_map_points.size()<<std::endl;
    if (_map_points.find(map_point->id_)==_map_points.end()) {
        _map_points.insert(std::make_pair(map_point->id_, map_point));
    }
    else
    {
        _map_points[map_point->id_] = map_point;
    }
    
}
} // myslam
