/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_CAMERA_SERVER_SHARED_H__
#define __CAUV_CAMERA_SERVER_SHARED_H__

#define SHMEM_SEGMENT_NAME "uk.co.cauv.shared.cameras"
#define CAMERA_SERVER_PORT 16708

namespace cauv{

typedef volatile uint32_t* lock_ptr;

// +ve IDs are OpenCV cameras, -ve IDs are libdc1394 cameras (if available)
// TODO: try to interpret IDs as dc1394 GUIDs first
struct ImageRequest{
    int32_t camera_id;
    uint32_t w;
    uint32_t h;
};

struct InfoResponse{
    // handle for SharedImage structure 
    uint64_t image_offset;
};

struct SharedImage{
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    int32_t type;
    // set to 0 when it is OK for the server to delete this image
    volatile int32_t lock;
    uint8_t bytes[1];
};


} // namespace cauv

#endif // ndef __CAUV_CAMERA_SERVER_SHARED_H__

