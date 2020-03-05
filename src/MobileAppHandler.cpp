#include <hal_mobile_app/MobileAppHandler.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#define ADDR_MASK 0x7FF
#define PROTOCOL_ARC_FRAME 0x035
#define PROTOCOL_HB_FRAME 0x001
#define PROTOCOL_HANDSHAKE_FRAME 0x00A
#define PROTOCOL_MANUALSTEERING 0x015
#define PROTOCOL_AUTOMATIC_STEERING 0x01F
#define PROTOCOL_ERROR_FRAMES 0x0C8
#define PROTOCOL_MAN_ERROR_FRAMES 0x0C9

#define LIN_VELO_SIGN_BYTE 0
#define DIRECTION_SIGN_BYTE 1
#define RADIUS_BYTE 2
#define LIN_VELO_BYTE 4
#define RADIUS_PARSE_CONST 10
#define LIN_VELO_PARSE_CONST 100

#define MAX_RADIUS 10
#define MIN_RADIUS 1.5

#define LEFT_ARCH 1
#define RIGHT_ARCH -1

#define BACKWARD -1
#define FORWARD 1

MobileAppHandler::MobileAppHandler(char* pchCanChannel)
{
    _CanHandler.iInitialize(pchCanChannel);
    _fRadius=0;
    _fManualLinearVelo=0;

    command_id = 0;
    fresh_data = false;
    chosen_task = "";
    int i = 0;
    while(i<8)
    {
        msg_byte_array[i] = 20;
        i++;
    }
}

void MobileAppHandler::vReceiveCanFrames()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        _FramesQueue.push(message);
    }
}

MobileAppHandler::~MobileAppHandler()
{
    while(!_FramesQueue.empty())
    {
        _FramesQueue.pop();
    }
}

int16_t MobileAppHandler::i16GetTwoBytes(uint8_t u8StartBit, can_frame CanFrame)
{
    return ((int16_t)CanFrame.data[u8StartBit])<<8 |((int16_t)CanFrame.data[u8StartBit+1]);
}

uint8_t MobileAppHandler::u8ParseCanFrames()
{
    uint8_t u8Status = 0;
    can_frame message;
    while(!_FramesQueue.empty())
    {
        message = _FramesQueue.front();
        if((PROTOCOL_ARC_FRAME&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            int8_t i8VeloSign = message.data[LIN_VELO_SIGN_BYTE] == 0 ? BACKWARD:FORWARD;
            int8_t i8DirectionSign = message.data[DIRECTION_SIGN_BYTE] == 0 ? RIGHT_ARCH:LEFT_ARCH;
            _fManualLinearVelo = ((float)(i16GetTwoBytes(LIN_VELO_BYTE, message))*i8VeloSign)/LIN_VELO_PARSE_CONST;
            _fRadius = ((float)(i16GetTwoBytes(RADIUS_BYTE, message))*i8DirectionSign)/RADIUS_PARSE_CONST;
            u8Status |= RECEIVED_MANUAL_STEERING;
        }
        else if((PROTOCOL_HANDSHAKE_FRAME&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_HANDSHAKE;
        } 
        else if((PROTOCOL_HB_FRAME&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_HEARTBEAT;
        }
        else if((PROTOCOL_MANUALSTEERING&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_MANUAL_MODE_COMF;
        }
        else if((PROTOCOL_AUTOMATIC_STEERING&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_AUTO_STEERING;
        }
        else if((PROTOCOL_ERROR_FRAMES&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_PROTOCOL_ERROR;
        }
        else if((PROTOCOL_MAN_ERROR_FRAMES&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_MANUALSTEERING_ERROR;
        } 
        _FramesQueue.pop();
    }
    return u8Status;
}

float MobileAppHandler::fGetManualRadius()
{
    return _fRadius;
}

float MobileAppHandler::fGetManualLinearVelo()
{
    return _fManualLinearVelo;
}

void MobileAppHandler::setSimulationAppFrame(const uint32_t rec_frame_ID, const boost::array<unsigned char, 8> rec_bytes)
{
    command_id=rec_frame_ID;
    msg_byte_array[0]=rec_bytes[0];
    msg_byte_array[1]=rec_bytes[1];
    msg_byte_array[2]=rec_bytes[2];
    msg_byte_array[3]=rec_bytes[3];
    msg_byte_array[4]=rec_bytes[4];
    msg_byte_array[5]=rec_bytes[5];
    msg_byte_array[6]=rec_bytes[6];
    msg_byte_array[7]=rec_bytes[7];
    ROS_INFO("Received app event ID = [%d]", rec_frame_ID);
    fresh_data=true;
}

//Method used to register and inform state machine about incoming msg
void MobileAppHandler::setSimulationMoveFrame(float fr_lin, float fr_ang, float bk_lin, float bk_ang)
{
    if (!fresh_data)
    {
        if (fr_lin != 0 && fr_ang != 0 && bk_lin != 0 && bk_ang != 0)
        {
            command_id=53;
            front_linear=fr_lin;
            front_angular=fr_ang;
            back_linear=bk_lin;
            back_angular=bk_ang;
            fresh_data=true;
            ROS_INFO("Received only move commands ID = [%d]", command_id);
        }
    } 
}

void MobileAppHandler::setChosenTask(std::string task_name)
{
    chosen_task = task_name;
}

void MobileAppHandler::createStateMachineAppFrame(agribot_msgs::AgribotMobileAppFrame* frame)
{
    if (fresh_data)
    {
        frame->header.stamp = ros::Time::now();
        frame->header.frame_id = "mobile_app_commands_link";
        frame->can_id = command_id;
        frame->can_byte_array[0] = msg_byte_array[0];
        frame->can_byte_array[1] = msg_byte_array[1];
        frame->can_byte_array[2] = msg_byte_array[2];
        frame->can_byte_array[3] = msg_byte_array[3];
        frame->can_byte_array[4] = msg_byte_array[4];
        frame->can_byte_array[5] = msg_byte_array[5];
        frame->can_byte_array[6] = msg_byte_array[6];
        frame->can_byte_array[7] = msg_byte_array[7];

        fresh_data = false;
    }
}


std::string MobileAppHandler::strgetTaskName()
{
    std::string temp = chosen_task;
    chosen_task="";
    return temp;
}

int MobileAppHandler::configure()
{
    return 0;
}

bool MobileAppHandler::startup()
{
    return true;
}

bool MobileAppHandler::update()
{
    return true;
}

bool MobileAppHandler::stop()
{
    return true;
}

bool MobileAppHandler::cleanup()
{
    return true;
}

bool MobileAppHandler::dataPendingSimulation()
{
    return fresh_data;
}