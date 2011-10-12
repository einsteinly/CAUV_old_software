#ifndef __CAUV_CAMERA_SERVER_SHARED_H__
#define __CAUV_CAMERA_SERVER_SHARED_H__

namespace cauv{

typedef volatile uint32_t* lock_ptr;

const static uint32_t Max_InfoResponse_Name_Len = 128;
struct InfoResponse{
    uint64_t lock_offset;
    uint64_t image_offset;
    uint32_t width;
    uint32_t height;
    int32_t type;
    char name[Max_InfoResponse_Name_Len];
};

} // namespace cauv

#endif // ndef __CAUV_CAMERA_SERVER_SHARED_H__

