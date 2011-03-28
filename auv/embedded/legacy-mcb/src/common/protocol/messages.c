#include "messages.h"

// ===============
// Message Structs
// ===============

void sendAsPacket(struct Message* m)
{
    switch (m->id) {
        case 0:
            sendDebugMessageAsPacket((struct DebugMessage*)m);
        case 2:
            sendMotorMessageAsPacket((struct MotorMessage*)m);
        case 40:
            sendAliveMessageAsPacket((struct AliveMessage*)m);
        case 85:
            sendResetMCBMessageAsPacket((struct ResetMCBMessage*)m);
        case 50:
            sendPressureMessageAsPacket((struct PressureMessage*)m);
    }
}

// debug group

struct DebugMessage emptyDebugMessage()
{
    struct DebugMessage m;
    m.id = 0;
    return m;
}
struct DebugMessage newDebugMessage(int8_t type, char* msg)
{
    struct DebugMessage m = emptyDebugMessage();
    m.type = type;
    m.msg = msg;
    return m;
}
struct DebugMessage DebugMessageFromPacket(struct packet p)
{
    struct DebugMessage m = emptyDebugMessage();
    int pos = 4; // Skip the message id (not as safe, but faster)
    pos = load_DebugType(p, pos, &m.type);
    pos = load_string(p, pos, &m.msg);
    
    return m;
}
void sendDebugMessageAsPacket(struct DebugMessage* m)
{
    int msgsize = 4; // 4 bytes for id
    msgsize += len_DebugType(&m->type);
    msgsize += len_string(&m->msg);

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;
    pos = save_uint32(p, pos, &m->id);
    pos = save_DebugType(p, pos, &m->type);
    pos = save_string(p, pos, &m->msg);

    sendPacket(p);
}

// control group

struct MotorMessage emptyMotorMessage()
{
    struct MotorMessage m;
    m.id = 2;
    return m;
}
struct MotorMessage newMotorMessage(int8_t motorId, int8_t speed)
{
    struct MotorMessage m = emptyMotorMessage();
    m.motorId = motorId;
    m.speed = speed;
    return m;
}
struct MotorMessage MotorMessageFromPacket(struct packet p)
{
    struct MotorMessage m = emptyMotorMessage();
    int pos = 4; // Skip the message id (not as safe, but faster)
    pos = load_MotorID(p, pos, &m.motorId);
    pos = load_int8(p, pos, &m.speed);
    
    return m;
}
void sendMotorMessageAsPacket(struct MotorMessage* m)
{
    int msgsize = 4; // 4 bytes for id
    msgsize += len_MotorID(&m->motorId);
    msgsize += len_int8(&m->speed);

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;
    pos = save_uint32(p, pos, &m->id);
    pos = save_MotorID(p, pos, &m->motorId);
    pos = save_int8(p, pos, &m->speed);

    sendPacket(p);
}

// mcb group

struct AliveMessage emptyAliveMessage()
{
    struct AliveMessage m;
    m.id = 40;
    return m;
}
struct AliveMessage newAliveMessage()
{
    struct AliveMessage m = emptyAliveMessage();
    return m;
}
struct AliveMessage AliveMessageFromPacket(struct packet p)
{
    struct AliveMessage m = emptyAliveMessage();
    return m;
}
void sendAliveMessageAsPacket(struct AliveMessage* m)
{
    int msgsize = 4; // 4 bytes for id

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;
    pos = save_uint32(p, pos, &m->id);

    sendPacket(p);
}

struct ResetMCBMessage emptyResetMCBMessage()
{
    struct ResetMCBMessage m;
    m.id = 85;
    return m;
}
struct ResetMCBMessage newResetMCBMessage()
{
    struct ResetMCBMessage m = emptyResetMCBMessage();
    return m;
}
struct ResetMCBMessage ResetMCBMessageFromPacket(struct packet p)
{
    struct ResetMCBMessage m = emptyResetMCBMessage();
    return m;
}
void sendResetMCBMessageAsPacket(struct ResetMCBMessage* m)
{
    int msgsize = 4; // 4 bytes for id

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;
    pos = save_uint32(p, pos, &m->id);

    sendPacket(p);
}

// pressure group

struct PressureMessage emptyPressureMessage()
{
    struct PressureMessage m;
    m.id = 50;
    return m;
}
struct PressureMessage newPressureMessage(uint16_t fore, uint16_t aft)
{
    struct PressureMessage m = emptyPressureMessage();
    m.fore = fore;
    m.aft = aft;
    return m;
}
struct PressureMessage PressureMessageFromPacket(struct packet p)
{
    struct PressureMessage m = emptyPressureMessage();
    int pos = 4; // Skip the message id (not as safe, but faster)
    pos = load_uint16(p, pos, &m.fore);
    pos = load_uint16(p, pos, &m.aft);
    
    return m;
}
void sendPressureMessageAsPacket(struct PressureMessage* m)
{
    int msgsize = 4; // 4 bytes for id
    msgsize += len_uint16(&m->fore);
    msgsize += len_uint16(&m->aft);

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;
    pos = save_uint32(p, pos, &m->id);
    pos = save_uint16(p, pos, &m->fore);
    pos = save_uint16(p, pos, &m->aft);

    sendPacket(p);
}



// =============
// Serialisation
// =============
//
int load_int32(struct packet p, int pos, int32_t* val)
{
    *val = *(int32_t*)(p.data+pos);
    return pos + sizeof(int32_t);
}
int save_int32(struct packet p, int pos, int32_t* val)
{
    *(int32_t*)(p.data+pos) = *val;
    return pos + sizeof(int32_t);
}
int len_int32(int32_t* val)
{
    return sizeof(int32_t);
}
int load_int16(struct packet p, int pos, int16_t* val)
{
    *val = *(int16_t*)(p.data+pos);
    return pos + sizeof(int16_t);
}
int save_int16(struct packet p, int pos, int16_t* val)
{
    *(int16_t*)(p.data+pos) = *val;
    return pos + sizeof(int16_t);
}
int len_int16(int16_t* val)
{
    return sizeof(int16_t);
}
int load_string(struct packet p, int pos, char** val)
{
    uint32_t len;
    pos = load_uint32(p,pos,&len);
    *val = *(char**)(p.data+pos);
    return pos + len;
}
int save_string(struct packet p, int pos, char** val)
{
    uint32_t len = strlen(*val);
    pos = save_uint32(p,pos,&len);
    memcpy(*val, p.data+pos, len);
    return pos + len;
}
int len_string(char** val)
{
    uint32_t len = strlen(*val);
    return len_uint32(&len) + len;
}
int load_double(struct packet p, int pos, double* val)
{
    *val = *(double*)(p.data+pos);
    return pos + sizeof(double);
}
int save_double(struct packet p, int pos, double* val)
{
    *(double*)(p.data+pos) = *val;
    return pos + sizeof(double);
}
int len_double(double* val)
{
    return sizeof(double);
}
int load_float(struct packet p, int pos, float* val)
{
    *val = *(float*)(p.data+pos);
    return pos + sizeof(float);
}
int save_float(struct packet p, int pos, float* val)
{
    *(float*)(p.data+pos) = *val;
    return pos + sizeof(float);
}
int len_float(float* val)
{
    return sizeof(float);
}
int load_uint8(struct packet p, int pos, uint8_t* val)
{
    *val = *(uint8_t*)(p.data+pos);
    return pos + sizeof(uint8_t);
}
int save_uint8(struct packet p, int pos, uint8_t* val)
{
    *(uint8_t*)(p.data+pos) = *val;
    return pos + sizeof(uint8_t);
}
int len_uint8(uint8_t* val)
{
    return sizeof(uint8_t);
}
int load_uint32(struct packet p, int pos, uint32_t* val)
{
    *val = *(uint32_t*)(p.data+pos);
    return pos + sizeof(uint32_t);
}
int save_uint32(struct packet p, int pos, uint32_t* val)
{
    *(uint32_t*)(p.data+pos) = *val;
    return pos + sizeof(uint32_t);
}
int len_uint32(uint32_t* val)
{
    return sizeof(uint32_t);
}
int load_uint16(struct packet p, int pos, uint16_t* val)
{
    *val = *(uint16_t*)(p.data+pos);
    return pos + sizeof(uint16_t);
}
int save_uint16(struct packet p, int pos, uint16_t* val)
{
    *(uint16_t*)(p.data+pos) = *val;
    return pos + sizeof(uint16_t);
}
int len_uint16(uint16_t* val)
{
    return sizeof(uint16_t);
}
int load_bool(struct packet p, int pos, char* val)
{
    *val = *(char*)(p.data+pos);
    return pos + sizeof(char);
}
int save_bool(struct packet p, int pos, char* val)
{
    *(char*)(p.data+pos) = *val;
    return pos + sizeof(char);
}
int len_bool(char* val)
{
    return sizeof(char);
}
int load_byte(struct packet p, int pos, uint8_t* val)
{
    *val = *(uint8_t*)(p.data+pos);
    return pos + sizeof(uint8_t);
}
int save_byte(struct packet p, int pos, uint8_t* val)
{
    *(uint8_t*)(p.data+pos) = *val;
    return pos + sizeof(uint8_t);
}
int len_byte(uint8_t* val)
{
    return sizeof(uint8_t);
}
int load_int8(struct packet p, int pos, int8_t* val)
{
    *val = *(int8_t*)(p.data+pos);
    return pos + sizeof(int8_t);
}
int save_int8(struct packet p, int pos, int8_t* val)
{
    *(int8_t*)(p.data+pos) = *val;
    return pos + sizeof(int8_t);
}
int len_int8(int8_t* val)
{
    return sizeof(int8_t);
}



int load_DebugType(struct packet p, int pos, int8_t* val)
{
    return load_int8(p, pos, val);
}
int save_DebugType(struct packet p, int pos, int8_t* val)
{
    return save_int8(p, pos, val);
}
int len_DebugType(int8_t* val)
{
    return len_int8(val);
}
int load_MotorID(struct packet p, int pos, int8_t* val)
{
    return load_int8(p, pos, val);
}
int save_MotorID(struct packet p, int pos, int8_t* val)
{
    return save_int8(p, pos, val);
}
int len_MotorID(int8_t* val)
{
    return len_int8(val);
}


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m)
{
    sendAsPacket(m);
}

void sendDebugMessage(int8_t type, char* msg)
{
    struct DebugMessage m = newDebugMessage(type, msg);
    sendDebugMessageAsPacket(&m);
}
void sendMotorMessage(int8_t motorId, int8_t speed)
{
    struct MotorMessage m = newMotorMessage(motorId, speed);
    sendMotorMessageAsPacket(&m);
}
void sendAliveMessage()
{
    struct AliveMessage m = newAliveMessage();
    sendAliveMessageAsPacket(&m);
}
void sendResetMCBMessage()
{
    struct ResetMCBMessage m = newResetMCBMessage();
    sendResetMCBMessageAsPacket(&m);
}
void sendPressureMessage(uint16_t fore, uint16_t aft)
{
    struct PressureMessage m = newPressureMessage(fore, aft);
    sendPressureMessageAsPacket(&m);
}

// ================
// Message Handlers
// ================

void handlePacket(struct packet p)
{
    uint32_t cid;
    load_uint32(p, 0, &cid);

    switch (cid) {
        case 0:
        {
	        struct DebugMessage m = DebugMessageFromPacket(p);
            handleDebugMessage(&m);
            break;
        }
        case 2:
        {
	        struct MotorMessage m = MotorMessageFromPacket(p);
            handleMotorMessage(&m);
            break;
        }
        case 40:
        {
	        struct AliveMessage m = AliveMessageFromPacket(p);
            handleAliveMessage(&m);
            break;
        }
        case 85:
        {
	        struct ResetMCBMessage m = ResetMCBMessageFromPacket(p);
            handleResetMCBMessage(&m);
            break;
        }
        case 50:
        {
	        struct PressureMessage m = PressureMessageFromPacket(p);
            handlePressureMessage(&m);
            break;
        }
        default:
            break;
    }
}

// Implementations of handleXMessage are separate
