#pragma once

#include <agribot_common/Can.h>
#include <queue>
#include <cmath>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <boost/array.hpp>
#include <string>

#define RECEIVED_HANDSHAKE 0x01
#define RECEIVED_HEARTBEAT 0x02
#define RECEIVED_MANUAL_MODE_COMF 0x04
#define RECEIVED_MANUAL_STEERING 0x08
#define RECEIVED_AUTO_STEERING 0x10
#define RECEIVED_PROTOCOL_ERROR 0x20
#define RECEIVED_MANUALSTEERING_ERROR 0x40

class MobileAppHandler
{
    Can _CanHandler;
    std::queue<can_frame> _FramesQueue;
    double _fManualLinearVelo = 0;
    double _fRadius = 0;
    int16_t i16GetTwoBytes(uint8_t u8StartBit, can_frame CanFrame);

    //simulation part
    bool fresh_data;
    int command_id;
    int8_t msg_byte_array[8];
    std::string chosen_task; 

    float front_linear;
    float front_angular;
    float back_linear;
    float back_angular;


public:
    MobileAppHandler(char* pchCanChannel);
    ~MobileAppHandler();
    void vReceiveCanFrames();
    uint8_t u8ParseCanFrames();
    float fGetManualRadius();
    float fGetManualLinearVelo();

    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();

    //read the chosen task name 
    void setChosenTask(std::string task_name);
    //returns the chosen task name 
    std::string strgetTaskName();

    //simulation part
    void createStateMachineAppFrame(agribot_msgs::AgribotMobileAppFrame* frame);

    void setSimulationAppFrame(const uint32_t rec_frame_ID, const boost::array<unsigned char, 8> rec_bytes);
    void setSimulationMoveFrame(float fr_lin, float fr_ang, float bk_lin, float bk_ang);


    //check if new simulation frame has been received
    bool dataPendingSimulation();
};