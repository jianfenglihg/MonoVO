#include "localmap.h"
namespace slam
{

void Localmap::addKeyFrame(Frame::Ptr frame)
{
    std::vector<Mappoint::Ptr>::iterator piter;
    std::vector<Frame::Ptr>::iterator fiter;
    if(key_frames_.size() == _max_key_frames)
    {
        std::vector<Frame::Ptr>::iterator iter = key_frames_.begin();
        long frame_id = (*iter)->id_;
        for (piter=map_points_.begin(); piter != map_points_.end(); piter++)
        {
            if((*piter)->_frame_id == frame_id)
            {
                map_points_.erase(piter);
            }
        }
        key_frames_.erase(iter);
        key_frames_.push_back(frame);
    }
    else
    {
        key_frames_.push_back(frame);
    }

    for (piter=map_points_.begin(); piter != map_points_.end(); piter++)
    {
        int flag = 0;
        for(fiter=key_frames_.begin(); fiter != key_frames_.end(); fiter++)
        {
            if((*piter)->_frame_id == (*fiter)->id_)
            {
                flag = 1;
            }
        }
        if(!flag) map_points_.erase(piter);   
    }

    
}
void Localmap::addMapPoint(Mappoint::Ptr point)
{
    map_points_.push_back(point);
}

} // namespace slam
