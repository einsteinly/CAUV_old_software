/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef FRAMES_H_
#define FRAMES_H_
#include <stdint.h>

#define dummy_MSG_CAN_ID 0

typedef union {
    uint8_t data[8];
    struct {
        uint8_t padding[8];
    } __attribute__((packed)) m;
} dummy_msg_t;

#define test_MSG_CAN_ID 1337

typedef union {
    uint8_t data[8];
    struct {
        struct {
            uint8_t bit1 : 1;
            uint8_t bit2 : 1;
            uint8_t _3bits : 3;
        } __attribute__((packed)) bitmap;
        uint8_t byte;
        float _float;
        int16_t _int;
    } __attribute__((packed)) m;
} test_msg_t;

#define bootload_MSG_CAN_ID 2

typedef union {
    uint8_t data[8];
    struct {
        uint16_t board_id;
        uint32_t image_size;
        uint8_t padding[2];
    } __attribute__((packed)) m;
} bootload_msg_t;

#define send_flash_MSG_CAN_ID 3

typedef union {
    uint8_t data[8];
    struct {
        uint32_t buf_size;
        uint8_t padding[4];
    } __attribute__((packed)) m;
} send_flash_msg_t;

#define image_data_MSG_CAN_ID 4

typedef union {
    uint8_t data[8];
    struct {
        uint8_t padding[8];
    } __attribute__((packed)) m;
} image_data_msg_t;

#define reset_MSG_CAN_ID 6

typedef union {
    uint8_t data[8];
    struct {
        uint16_t board_id;
        uint8_t padding[6];
    } __attribute__((packed)) m;
} reset_msg_t;

#define debug_enable_MSG_CAN_ID 7

typedef union {
    uint8_t data[8];
    struct {
        uint16_t board_id;
        uint8_t enable;
        uint8_t padding[5];
    } __attribute__((packed)) m;
} debug_enable_msg_t;

#define debug_output_MSG_CAN_ID 107

typedef union {
    uint8_t data[8];
    struct {
        double dummy;
    } __attribute__((packed)) m;
} debug_output_msg_t;

#define flash_verify_MSG_CAN_ID 5

typedef union {
    uint8_t data[8];
    struct {
        uint32_t crc32;
        uint8_t padding[4];
    } __attribute__((packed)) m;
} flash_verify_msg_t;

#define pressure_MSG_CAN_ID 10

typedef union {
    uint8_t data[8];
    struct {
        int16_t pressure;
        int16_t temperature;
        uint8_t position;
        uint8_t padding[3];
    } __attribute__((packed)) m;
} pressure_msg_t;

#define motor_cmd_MSG_CAN_ID 20

typedef union {
    uint8_t data[8];
    struct {
        int8_t fwd_left;
        int8_t fwd_right;
        int8_t vert_fore;
        int8_t vert_aft;
        int8_t horz_fore;
        int8_t horz_aft;
        uint8_t padding[2];
    } __attribute__((packed)) m;
} motor_cmd_msg_t;

#define motor_status_MSG_CAN_ID 21

typedef union {
    uint8_t data[8];
    struct {
        struct {
            uint8_t otw_vert : 1;
            uint8_t otw_horz : 1;
            uint8_t otw_fwd : 1;
            uint8_t fault_vert : 1;
            uint8_t fault_horz : 1;
            uint8_t fault_fwd : 1;
            uint8_t timeout : 1;
        } __attribute__((packed)) flags;
        uint8_t padding[7];
    } __attribute__((packed)) m;
} motor_status_msg_t;

#define motor_settings_MSG_CAN_ID 22

typedef union {
    uint8_t data[8];
    struct {
        uint16_t slew_time;
        uint8_t padding[6];
    } __attribute__((packed)) m;
} motor_settings_msg_t;

#define front_light_level_MSG_CAN_ID 30

typedef union {
    uint8_t data[8];
    struct {
        uint8_t level;
        uint8_t padding[7];
    } __attribute__((packed)) m;
} front_light_level_msg_t;

#define down_light_level_MSG_CAN_ID 31

typedef union {
    uint8_t data[8];
    struct {
        uint8_t level;
        uint8_t padding[7];
    } __attribute__((packed)) m;
} down_light_level_msg_t;

#define strobe_light_level_MSG_CAN_ID 32

typedef union {
    uint8_t data[8];
    struct {
        uint8_t level;
        uint8_t time_period;
        uint8_t duty_cycle;
        uint8_t padding[5];
    } __attribute__((packed)) m;
} strobe_light_level_msg_t;

#define msb_f_init_MSG_CAN_ID 40

typedef union {
    uint8_t data[8];
    struct {
        uint8_t padding[8];
    } __attribute__((packed)) m;
} msb_f_init_msg_t;



typedef struct {
    void (*dummy)(dummy_msg_t*, void*);
    void *dummy_priv;
    void (*test)(test_msg_t*, void*);
    void *test_priv;
    void (*bootload)(bootload_msg_t*, void*);
    void *bootload_priv;
    void (*send_flash)(send_flash_msg_t*, void*);
    void *send_flash_priv;
    void (*image_data)(image_data_msg_t*, void*);
    void *image_data_priv;
    void (*reset)(reset_msg_t*, void*);
    void *reset_priv;
    void (*debug_enable)(debug_enable_msg_t*, void*);
    void *debug_enable_priv;
    void (*flash_verify)(flash_verify_msg_t*, void*);
    void *flash_verify_priv;
    void (*_default)(can_frame*, void*);
    void *_default_priv;
} frame_callbacks_t;

int frame_callback_loop (int canfd, frame_callbacks_t *callbacks);
#endif
