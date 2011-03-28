#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdlib.h>
#include <string.h>

// ================
// Type definitions
// ================

typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;



#define CAUV_DEBUGTYPE_TRACE 0
#define CAUV_DEBUGTYPE_DEBUG 1
#define CAUV_DEBUGTYPE_ERROR 2
#define CAUV_DEBUGTYPE_NUMVALUES 3

#define CAUV_MOTORID_PROP 1
#define CAUV_MOTORID_HBOW 2
#define CAUV_MOTORID_VBOW 4
#define CAUV_MOTORID_HSTERN 8
#define CAUV_MOTORID_VSTERN 16
#define CAUV_MOTORID_NUMVALUES 5


// ===============
// Message Structs
// ===============

struct packet {
    unsigned char* data;
    uint32_t length;
};
void destroyPacket(struct packet* p);

// Base message struct
struct Message
{
    uint32_t id;
};
void sendAsPacket(struct Message* m);

// debug group

struct DebugMessage
{
    uint32_t id;
    
    int8_t type;
    char* msg;
};

struct DebugMessage emptyDebugMessage();
struct DebugMessage newDebugMessage(int8_t type, char* msg);
struct DebugMessage DebugMessageFromPacket(struct packet p);
void sendDebugMessageAsPacket(struct DebugMessage* m);

// control group

struct MotorMessage
{
    uint32_t id;
    
    int8_t motorId;
    int8_t speed;
};

struct MotorMessage emptyMotorMessage();
struct MotorMessage newMotorMessage(int8_t motorId, int8_t speed);
struct MotorMessage MotorMessageFromPacket(struct packet p);
void sendMotorMessageAsPacket(struct MotorMessage* m);

// mcb group

struct AliveMessage
{
    uint32_t id;
    
};

struct AliveMessage emptyAliveMessage();
struct AliveMessage newAliveMessage();
struct AliveMessage AliveMessageFromPacket(struct packet p);
void sendAliveMessageAsPacket(struct AliveMessage* m);

struct ResetMCBMessage
{
    uint32_t id;
    
};

struct ResetMCBMessage emptyResetMCBMessage();
struct ResetMCBMessage newResetMCBMessage();
struct ResetMCBMessage ResetMCBMessageFromPacket(struct packet p);
void sendResetMCBMessageAsPacket(struct ResetMCBMessage* m);

// pressure group

struct PressureMessage
{
    uint32_t id;
    
    uint16_t fore;
    uint16_t aft;
};

struct PressureMessage emptyPressureMessage();
struct PressureMessage newPressureMessage(uint16_t fore, uint16_t aft);
struct PressureMessage PressureMessageFromPacket(struct packet p);
void sendPressureMessageAsPacket(struct PressureMessage* m);


// =============
// Serialisation
// =============
//
int load_int32(struct packet p, int pos, int32_t* val);
int save_int32(struct packet p, int pos, int32_t* val);
int len_int32(int32_t* val);
int load_int16(struct packet p, int pos, int16_t* val);
int save_int16(struct packet p, int pos, int16_t* val);
int len_int16(int16_t* val);
int load_string(struct packet p, int pos, char** val);
int save_string(struct packet p, int pos, char** val);
int len_string(char** val);
int load_double(struct packet p, int pos, double* val);
int save_double(struct packet p, int pos, double* val);
int len_double(double* val);
int load_float(struct packet p, int pos, float* val);
int save_float(struct packet p, int pos, float* val);
int len_float(float* val);
int load_uint8(struct packet p, int pos, uint8_t* val);
int save_uint8(struct packet p, int pos, uint8_t* val);
int len_uint8(uint8_t* val);
int load_uint32(struct packet p, int pos, uint32_t* val);
int save_uint32(struct packet p, int pos, uint32_t* val);
int len_uint32(uint32_t* val);
int load_uint16(struct packet p, int pos, uint16_t* val);
int save_uint16(struct packet p, int pos, uint16_t* val);
int len_uint16(uint16_t* val);
int load_bool(struct packet p, int pos, char* val);
int save_bool(struct packet p, int pos, char* val);
int len_bool(char* val);
int load_byte(struct packet p, int pos, uint8_t* val);
int save_byte(struct packet p, int pos, uint8_t* val);
int len_byte(uint8_t* val);
int load_int8(struct packet p, int pos, int8_t* val);
int save_int8(struct packet p, int pos, int8_t* val);
int len_int8(int8_t* val);



int load_DebugType(struct packet p, int pos, int8_t* val);
int save_DebugType(struct packet p, int pos, int8_t* val);
int len_DebugType(int8_t* val);
int load_MotorID(struct packet p, int pos, int8_t* val);
int save_MotorID(struct packet p, int pos, int8_t* val);
int len_MotorID(int8_t* val);


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m);

void sendDebugMessage(int8_t type, char* msg);
void sendMotorMessage(int8_t motorId, int8_t speed);
void sendAliveMessage();
void sendResetMCBMessage();
void sendPressureMessage(uint16_t fore, uint16_t aft);

// ================
// Message Handlers
// ================

void handlePacket(struct packet p);

void handleDebugMessage(struct DebugMessage* m);
void handleMotorMessage(struct MotorMessage* m);
void handleAliveMessage(struct AliveMessage* m);
void handleResetMCBMessage(struct ResetMCBMessage* m);
void handlePressureMessage(struct PressureMessage* m);


#endif//__MESSAGES_H__
