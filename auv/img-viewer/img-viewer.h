#ifndef __CAUV_IMG_VIEWER_H__
#define __CAUV_IMG_VIEWER_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <camera/camera.h>

namespace cauv{

class ImageViewer : public CauvNode
{
    public:
        ImageViewer(const CameraID::e camera_id, const int device_id);
};

} // namespace cauv

#endif//__CAUV_IMG_VIEWER_H__
