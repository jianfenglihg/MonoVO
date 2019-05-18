#ifndef LOCALMAP_H
#define LOCALMAP_H

#include "common_include.h"
#include "map.h"
#include "frame.h"
#include "mappoint.h"

namespace slam
{

class Localmap
{
public:
    int _max_key_frames;
    std::vector<Frame::Ptr> key_frames_;
    std::vector<Mappoint::Ptr> map_points_;
public:
    typedef std::shared_ptr<Localmap> Ptr;
    Localmap() {}
    ~Localmap() {}
    Localmap(int max_key_frames)
    :_max_key_frames(max_key_frames)
    {

    }
    void addKeyFrame(Frame::Ptr frame);
    void addMapPoint(Mappoint::Ptr point);
};
} // namespace slam




#endif