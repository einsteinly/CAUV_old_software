#ifndef __GEMINIDEFAULTLIMITS_H__
#define __GEMINIDEFAULTLIMITS_H__

// Constants

#define PACKET_RETRY_COUNT                6

#define geminiCommsModeEvo                1
#define geminiCommsModeSeaNet             2 
#define geminiCommsModeProd               3
#define geminiCommsModeSeaNetComp         4 
#define geminiCommsModeEvoComp            5 

#define geminiBaud2400                    5208
#define geminiBaud4800                    2604
#define geminiBaud9600                    1302
#define geminiBaud19200                   651
#define geminiBaud38400                   325
#define geminiBaud57600                   217
#define geminiBaud115200                  108

#define geminiPollPeriodFor2400Baud       50000
#define geminiPollPeriodFor4800Baud       50000
#define geminiPollPeriodFor9600Baud       50000
#define geminiPollPeriodFor19200Baud      50000
#define geminiPollPeriodFor38400Baud      25000
#define geminiPollPeriodFor57600Baud      12500
#define geminiPollPeriodFor115200Baud      6250

// Default values

#define geminiCommsModeDefault            geminiCommsModeEvo
#define geminiEvoRangeCompressionMax      7
#define geminiEvoRangeCompressionDefault  6

#define geminiPingShadeBankDefault        1

#define geminiPingStartBeamDefault        0
#define geminiPingEndBeamDefault          255

#define geminiPingAbsorbtionGainDefault   0
#define geminiPingSpeadingGainDefault     250

#define geminiPingPowerManagementDefault  1
#define geminiSoftwareGainRampDefault     0
#define geminiVelocimeterTXLengthDefault  1
#define geminiVelocimeterTXPeriodDefault  30

#define geminiMaximumRange                150
#define geminiMaximumRangeInLines         ((geminiMaximumRange * 125) + 250) // 250 added just some slack

#if 1
#define geminiVelocimeterTXMarkDefault    17
#else
#define geminiVelocimeterTXMarkDefault    6

#pragma message ("**********************************************")
#pragma message ("geminiVelocimeterTXMarkDefault special for 513")
#pragma message ("**********************************************")
#endif

#define geminiTXMarkDefault               29

// Limit values

#define geminiPingStartBeamMin            0
#define geminiPingStartBeamMax            255

#define geminiPingEndBeamMin              0
#define geminiPingEndBeamMax              255

#define geminiInterMessageGapMin          12
#define geminiInterMessageGapMax          255

// VDSL Settings

#define geminiVDSLSettings1DSDataRate     50
#define geminiVDSLSettings1USDataRate     20
#define geminiVDSLSettings1DSSNR1         10
#define geminiVDSLSettings1DSSNR2         10
#define geminiVDSLSettings1USSNR1         10
#define geminiVDSLSettings1USSNR2         10
#define geminiVDSLSettings1DSInterleave   10
#define geminiVDSLSettings1USInterleave   10

#define geminiVDSLSettings2DSDataRate     45
#define geminiVDSLSettings2USDataRate     15
#define geminiVDSLSettings2DSSNR1         12
#define geminiVDSLSettings2DSSNR2         12
#define geminiVDSLSettings2USSNR1         12
#define geminiVDSLSettings2USSNR2         12
#define geminiVDSLSettings2DSInterleave   22
#define geminiVDSLSettings2USInterleave   22

#define geminiVDSLSettings3DSDataRate     45
#define geminiVDSLSettings3USDataRate     15
#define geminiVDSLSettings3DSSNR1         14
#define geminiVDSLSettings3DSSNR2         14
#define geminiVDSLSettings3USSNR1         14
#define geminiVDSLSettings3USSNR2         14
#define geminiVDSLSettings3DSInterleave   32
#define geminiVDSLSettings3USInterleave   32
#endif //__GEMINIDEFAULTLIMITS_H__

