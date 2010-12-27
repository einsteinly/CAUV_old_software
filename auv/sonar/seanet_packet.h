#ifndef __CAUV_SEANET_PACKET_H__
#define __CAUV_SEANET_PACKET_H__

#include <string>
#include <stdint.h>

namespace cauv{

namespace SeanetMessageType
{
    enum e
    {                
        Null = 0,
        VersionData = 1,
        HeadData = 2,
        SpectData = 3,
        Alive = 4,
        PrgAck = 5,
        BBUserData = 6,
        TestData = 7,
        AuxData = 8,
        AdcData = 9,
        AdcReq = 10,
        LanStatus = 13,
        SetTime = 14,
        Timeout = 15,
        ReBoot = 16,
        PerformanceData = 17,
        HeadCommand = 19,
        EraseSector = 20,
        ProgBlock = 21,
        CopyBootBlk = 22,
        SendVersion = 23,
        SendBBuser = 24,
        SendData = 25,
        SendPerfnceData = 26,
        FPGAVersData = 57,
        FPGACalibData = 63
    };
}

namespace HdCtrl
{
    enum e
    {
        Adc8on =       1<<0,   // 4/8 bit res?
        Continuous =   1<<1,   // Continuous scanning?        
        ScanRight =    1<<2,   // Scan direction              
        Invert =       1<<3,   // Invert scan direction       
        Motoff =       1<<4,   // Disable motor               
        Txoff =        1<<5,   // Disable sonar transmitter   
        Spare =        1<<6,   // 0 always                    
        Chan2 =        1<<7,   // Use channel 2?              
        Raw =          1<<8,   // 1 always                    
        HasMot =       1<<9,   // 1 always (we've got a motor)
        ApplyOffset =  1<<10,  // Apply heading offsets?      
        PingPong =     1<<11,  // 0 always (used for sidescan)
        StareLLim =    1<<12,  // 0 always (seasprite can't)  
        ReplyASL =     1<<13,  // 1 always                    
        ReplyThr =     1<<14,  // 0 always                    
        IgnoreSensor = 1<<15,  // 0 always                    
    };
}


namespace HdInf
{
    enum e
    {
        InCentre = 1<<0, // Transducer is in middle of a re-centre operation.
        Centred =  1<<1, // Transducer is at centre position ready to start scan.
        Motoring = 1<<2, // Motor is motoring (can be re-centring or in scan).
        MotorOn =  1<<3, // Device is powered and Motor is primed.
        Dir =      1<<4, // Transducer is off-centre.
        InScan =   1<<5, // Device is currently performing a Scan/Ping routine.
        NoParams = 1<<6, // Device needs Parameters before it can Scan/Ping.
        SentCfg =  1<<7, // Acknowledgement of 'Params' received.
    };
}

/*
 * A mtHeadData message has this as it's body, followed by 'dBytes' worth
 * of data bins.
 */
class SeanetHeadData {
    public:
        unsigned char messageSequence;
        unsigned char txNode;
        uint16_t byteCount;
        unsigned char dev_type;
        unsigned char hdStatus;
        unsigned char sweep;
        uint16_t hdCtrl;
        uint16_t rangeScale;
        uint32_t txNumber;
        unsigned char gain;
        uint16_t slope;
        unsigned char adSpan;
        unsigned char adLow;
        uint16_t headingOffset;
        uint16_t adInterval;
        uint16_t leftLim;
        uint16_t rightLim;
        unsigned char stepSize;
        uint16_t bearing;
        uint16_t dBytes;
        unsigned char bin1;

        static bool same_settings(const SeanetHeadData& first, const SeanetHeadData& second);
} __attribute__((__packed__));

/*
 * WARNING
 * Don't add virtual functions to this class. I've no idea if the pointer
 * to the VFT will end up messing up the aligment of the members.
 */
struct SeanetHeadParams {
    public:
        unsigned char v3bParams;
        uint16_t hdCtrl;
        unsigned char hdType;
        uint32_t txnChn1;
        uint32_t txnChn2;
        uint32_t rxnChn1;
        uint32_t rxnChn2;
        uint16_t txPulseLen;
        uint16_t rangeScale;
        uint16_t leftLim;
        uint16_t rightLim;
        unsigned char adSpan;
        unsigned char adLow;
        unsigned char igainChn1;
        unsigned char igainChn2;
        uint16_t slopeChn1;
        uint16_t slopeChn2;
        unsigned char moTime;
        unsigned char stepSize;
        uint16_t adInterval;
        uint16_t nBins;
        uint16_t maxAdBuf;
        uint16_t lockout;
        uint16_t minAxisDir;
        unsigned char majAxisPan;
        unsigned char ctl2;
        uint16_t scanZ;
        /* v3b params 
        unsigned char v3b_adSpnChn1;
        unsigned char v3b_adSpnChn2;
        unsigned char v3b_adLowChn1;
        unsigned char v3b_adLowChn2;
        unsigned char v3b_igainChn1;
        unsigned char v3b_igainChn2;
        unsigned char v3b_adcSetPChn1;
        unsigned char v3b_adcSetPChn2;
        uint16_t v3b_slopeChn1;
        uint16_t v3b_slopeChn2;
        uint16_t v3b_slopeDelayChn1;
        uint16_t v3b_slopeDelayChn2;
    */
    public:
        SeanetHeadParams();
} __attribute__((__packed__));

class SeanetPacket {
    protected:
        /* Length of payload not inc LF */
        unsigned short m_length;
        /* Source identifier */
        unsigned char m_sid;
        /* Desination identifier */
        unsigned char m_did;

        unsigned char m_count;
        
        unsigned char m_type;

        /* The full data of the packet */
        std::string m_data;

        void fillHeader();
    public:
        const std::string& getData() const;
        unsigned char getType() const;
        unsigned short getLength() const;

        friend class SeanetSerialPort;
};

class SeanetHeadParamsPacket : public SeanetPacket {
public:
	SeanetHeadParamsPacket(SeanetHeadParams &params);
};

class SeanetRebootPacket : public SeanetPacket {
public:
	SeanetRebootPacket();
};

class SeanetSendBBUserPacket : public SeanetPacket {
public:
	SeanetSendBBUserPacket();
};

class SeanetSendDataPacket : public SeanetPacket {
public:
	SeanetSendDataPacket();
};

class SeanetSendVersionPacket : public SeanetPacket {
public:
	SeanetSendVersionPacket();
};

} // namespace cauv

#endif // ndef __CAUV_SEANET_PACKET_H__

