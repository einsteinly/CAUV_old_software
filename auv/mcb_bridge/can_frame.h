#ifndef CAN_FRAME_H_
#define CAN_FRAME_H_

#include <stdint.h>

typedef struct { //changes to this should be synced with msg_template in gen_frames.py
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} __attribute__((packed)) can_frame_t;

#endif
