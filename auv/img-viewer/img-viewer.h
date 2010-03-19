#ifndef __IMG_VIEWER_H__
#define __IMG_VIEWER_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <camera/camera.h>

class ImageViewer : public CauvNode
{
    public:
        ImageViewer(const CameraID camera_id, const int device_id);
};

#endif//__IMG_VIEWER_H__
