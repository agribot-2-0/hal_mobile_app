#pragma once

#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_common/Can.h>
#include <queue>
#include <cmath>
#include <string>
#include <boost/array.hpp>

#define RECEIVED_HANDSHAKE 0x01
#define RECEIVED_HEARTBEAT 0x02
#define RECEIVED_MANUAL_MODE_COMF 0x04
#define RECEIVED_MANUAL_STEERING 0x08
#define RECEIVED_AUTO_STEERING 0x10
#define RECEIVED_PROTOCOL_ERROR 0x20
#define RECEIVED_MANUALSTEERING_ERROR 0x40
#define RECEIVED_CONFIGURATION 0x80

class MobileAppHandler
{
    Can _CanHandler;
    std::queue<can_frame> _FramesQueue;
    double _fManualLinearVelo = 0;
    double _fRadius = 0;
    std::queue<uint8_t> _ProtocolErrorTypeQueue;
    std::queue<uint8_t> _ProtocolErrorSpecificQueue;
    std::queue<uint8_t> _ManualErrorTypeQueue;
    std::queue<uint8_t> _ManualErrorSpecificQueue;
    int16_t i16GetTwoBytes(uint8_t u8StartBit, can_frame CanFrame);

    // onfiguration data
    bool bReceivedCompleteConfig;
    uint uFrontToolAngle;
    uint uFrontToolLength;
    uint8_t u8FrontToolValveRedFlow;
    uint8_t u8FrontToolValveGreenFlow;
    uint8_t u8FrontToolValveBlueFlow;
    uint8_t u8FrontToolMotorSpeed;
    uint uBackToolAngle;
    uint uBackToolLength;
    uint8_t u8BackToolValveRedFlow;
    uint8_t u8BackToolValveGreenFlow;
    uint8_t u8BackToolValveBlueFlow;
    uint8_t u8BackToolMotorSpeed;

    // simulation part
    bool bFreshData;
    int iCommandId;
    int8_t msg_byte_array[8];
    std::string strChosenTask;

    float fFrontLinear;
    float fFrontAngular;
    float fBackLinear;
    float fBackAngular;


 public:
    explicit MobileAppHandler(char* pchCanChannel);
    ~MobileAppHandler();
    void vReceiveCanFrames();
    uint8_t u8ParseCanFrames();
    float fGetManualRadius();
    float fGetManualLinearVelo();
    void vAssemblyStateMachineFrame(
        agribot_msgs::AgribotMobileAppFrame* pFrameMsg,
        uint uFrameType);

    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();

    // read the chosen task name
    void vSetChosenTask(std::string strTaskName);
    // returns the chosen task name
    std::string strGetTaskName();

    // simulation part
    void vCreateSimStateMachineAppFrame(
        agribot_msgs::AgribotMobileAppFrame* pFrame);

    void vSetSimulationAppFrame(const uint32_t uRecFrameID,
        const boost::array<unsigned char, 8> _chRecBytes);
    void vSetSimulationMoveFrame(float fFrLin,
        float fFrAng, float fBkLin, float fBkAng);

    // check if new simulation frame has been received
    bool bDataPendingSimulation();
};
