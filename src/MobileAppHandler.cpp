#include <hal_mobile_app/MobileAppHandler.h>
#include <iostream>

#define ADDR_MASK 0x7FF
#define PROTOCOL_ARC_FRAME 0x035
#define PROTOCOL_HB_FRAME 0x001
#define PROTOCOL_HANDSHAKE_FRAME 0x00A
#define PROTOCOL_MANUALSTEERING 0x015
#define PROTOCOL_AUTOMATIC_STEERING 0x01F
#define PROTOCOL_ERROR_FRAMES 0x0C8
#define PROTOCOL_MAN_ERROR_FRAMES 0x0C9
#define PROTOCOL_CONFIGURATION_FRAME 0x96

#define LIN_VELO_SIGN_BYTE 0
#define DIRECTION_SIGN_BYTE 1
#define RADIUS_BYTE 2
#define LIN_VELO_BYTE 4
#define RADIUS_PARSE_CONST 10
#define LIN_VELO_PARSE_CONST 100

#define CONFIG_TYPE_BYTE 0
#define CONFIG_FRONT_TOOL_AL 1
#define CONFIG_FRONT_TOOL_VM 2
#define CONFIG_BACK_TOOL_AL 3
#define CONFIG_BACK_TOOL_VM 4

#define CONFIG_TOOL_ANGLE_BYTE 1
#define CONFIG_TOOL_LENGTH_BYTE 3

#define CONFIG_TOOL_VALVE_R_BYTE 1
#define CONFIG_TOOL_VALVE_G_BYTE 2
#define CONFIG_TOOL_VALVE_B_BYTE 3
#define CONFIG_TOOL_MOTOR_BYTE 4

#define MAX_RADIUS 10
#define MIN_RADIUS 1.5

#define LEFT_ARCH 1
#define RIGHT_ARCH -1

#define BACKWARD -1
#define FORWARD 1

#define HEADER_STRING_MOB_APP_FRAME "mobile_app_commands_link"
#define ERROR_TYPE_BYTE 0
#define ERROR_SPECIFIC_BYTE 1


MobileAppHandler::MobileAppHandler(char* pchCanChannel)
{
    CanHandler.iInitialize(pchCanChannel);
    dRadius = 0;
    dManualLinearVelo = 0;

    iCommandId = 0;
    bFreshData = false;
    strChosenTask = "";
    int i = 0;
    while (i < 8)
    {
        msg_byte_array[i] = 20;
        i++;
    }
    bReceivedCompleteConfig = false;
    uFrontToolAngle = 0;
    uFrontToolLength = 0;
    u8FrontToolValveRedFlow = 0;
    u8FrontToolValveGreenFlow = 0;
    u8FrontToolValveBlueFlow = 0;
    u8FrontToolMotorSpeed = 0;
    uBackToolAngle = 0;
    uBackToolLength = 0;
    u8BackToolValveRedFlow = 0;
    u8BackToolValveGreenFlow = 0;
    u8BackToolValveBlueFlow = 0;
    u8BackToolMotorSpeed = 0;
}

void MobileAppHandler::vReceiveCanFrames()
{
    can_frame message;
    while (CanHandler.bReceiveFrame(&message))
    {
        FramesQueue.push(message);
    }
}

MobileAppHandler::~MobileAppHandler()
{
    while (!FramesQueue.empty())
    {
        FramesQueue.pop();
    }
}

int16_t MobileAppHandler::i16GetTwoBytes(
    uint8_t u8StartBit,
     can_frame CanFrame)
{
    return ((int16_t)CanFrame.data[u8StartBit]) <<8 |
           ((int16_t)CanFrame.data[u8StartBit+1]);
}

uint8_t MobileAppHandler::u8ParseCanFrames()
{
    uint8_t u8Status = 0;
    can_frame message;
    while (!FramesQueue.empty())
    {
        message = FramesQueue.front();
        if ((PROTOCOL_ARC_FRAME&ADDR_MASK) == (message.can_id&ADDR_MASK))
        {
            int8_t i8VeloSign =
                message.data[LIN_VELO_SIGN_BYTE] == 0 ? BACKWARD:FORWARD;
            int8_t i8DirectionSign = message.data[DIRECTION_SIGN_BYTE]
                == 0 ? RIGHT_ARCH:LEFT_ARCH;
            dManualLinearVelo =
                (static_cast<float>(i16GetTwoBytes(LIN_VELO_BYTE, message))*
                i8VeloSign) / LIN_VELO_PARSE_CONST;
            dRadius = (static_cast<float>
                (i16GetTwoBytes(RADIUS_BYTE, message)) * i8DirectionSign)
                / RADIUS_PARSE_CONST;
            u8Status |= RECEIVED_MANUAL_STEERING;
        }
        else if ((PROTOCOL_HANDSHAKE_FRAME&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_HANDSHAKE;
        }
        else if ((PROTOCOL_HB_FRAME&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_HEARTBEAT;
        }
        else if ((PROTOCOL_MANUALSTEERING&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_MANUAL_MODE_COMF;
        }
        else if ((PROTOCOL_AUTOMATIC_STEERING&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            u8Status |= RECEIVED_AUTO_STEERING;
        }
        else if ((PROTOCOL_ERROR_FRAMES&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            ProtocolErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
            ProtocolErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
            u8Status |= RECEIVED_PROTOCOL_ERROR;
        }
        else if ((PROTOCOL_MAN_ERROR_FRAMES&ADDR_MASK) ==
                 (message.can_id&ADDR_MASK))
        {
            ManualErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
            ManualErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
            u8Status |= RECEIVED_MANUALSTEERING_ERROR;
        }
        else if ((PROTOCOL_CONFIGURATION_FRAME&ADDR_MASK)
                 == (message.can_id&ADDR_MASK))
        {
            if (message.data[CONFIG_TYPE_BYTE] == CONFIG_FRONT_TOOL_AL)
            {
                uFrontToolAngle = static_cast<uint>
                (i16GetTwoBytes(CONFIG_TOOL_ANGLE_BYTE, message));
                uFrontToolLength = static_cast<uint>
                (i16GetTwoBytes(CONFIG_TOOL_LENGTH_BYTE, message));
            }
            else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_FRONT_TOOL_VM)
            {
                u8FrontToolValveRedFlow =
                message.data[CONFIG_TOOL_VALVE_R_BYTE];
                u8FrontToolValveGreenFlow =
                message.data[CONFIG_TOOL_VALVE_G_BYTE];
                u8FrontToolValveBlueFlow =
                message.data[CONFIG_TOOL_VALVE_B_BYTE];
                u8FrontToolMotorSpeed =
                message.data[CONFIG_TOOL_MOTOR_BYTE];
            }
            else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_BACK_TOOL_AL)
            {
                uBackToolAngle = static_cast<uint>
                (i16GetTwoBytes(CONFIG_TOOL_ANGLE_BYTE, message));
                uBackToolLength = static_cast<uint>
                (i16GetTwoBytes(CONFIG_TOOL_LENGTH_BYTE, message));
            }
            else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_BACK_TOOL_VM)
            {
                u8BackToolValveRedFlow =
                message.data[CONFIG_TOOL_VALVE_R_BYTE];
                u8BackToolValveGreenFlow =
                message.data[CONFIG_TOOL_VALVE_G_BYTE];
                u8BackToolValveBlueFlow =
                message.data[CONFIG_TOOL_VALVE_B_BYTE];
                u8BackToolMotorSpeed =
                message.data[CONFIG_TOOL_MOTOR_BYTE];
            }
            u8Status |= RECEIVED_CONFIGURATION;
        }
        FramesQueue.pop();
    }
    return u8Status;
}

void MobileAppHandler::vAssemblyStateMachineFrame(
    agribot_msgs::AgribotMobileAppFrame* ptrFrameMsg, uint uFrameType)
{
    switch (uFrameType)
    {
        case RECEIVED_MANUAL_STEERING:
            ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
            ptrFrameMsg->can_id = PROTOCOL_ARC_FRAME;
            if (dManualLinearVelo >= 0)
                ptrFrameMsg->can_byte_array[LIN_VELO_SIGN_BYTE] = 1;
            else
                ptrFrameMsg->can_byte_array[LIN_VELO_SIGN_BYTE] = 0;
            ptrFrameMsg->can_byte_array[LIN_VELO_BYTE] =
            ((abs(static_cast<int>(dManualLinearVelo*100)))&0xFF00)>>8;
            ptrFrameMsg->can_byte_array[LIN_VELO_BYTE+1] =
            (abs(static_cast<int>(dManualLinearVelo*100)))&0x00FF;
            if (dRadius >= 0)
                ptrFrameMsg->can_byte_array[DIRECTION_SIGN_BYTE] = 1;
            else
                ptrFrameMsg->can_byte_array[DIRECTION_SIGN_BYTE] = 0;
            ptrFrameMsg->can_byte_array[RADIUS_BYTE] =
            ((abs(static_cast<int>(dRadius*10)))&0xFF00)>>8;
            ptrFrameMsg->can_byte_array[RADIUS_BYTE+1] =
            (abs(static_cast<int>(dRadius*10)))&0x00FF;
            break;
        case RECEIVED_HANDSHAKE:
            ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
            ptrFrameMsg->can_id = PROTOCOL_HANDSHAKE_FRAME;
            ptrFrameMsg->can_byte_array[0] = 10;   // dummy load
            break;
        case RECEIVED_MANUAL_MODE_COMF:
            ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
            ptrFrameMsg->can_id = PROTOCOL_MANUALSTEERING;
            ptrFrameMsg->can_byte_array[0] = 10;
            break;
        case RECEIVED_AUTO_STEERING:
            ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
            ptrFrameMsg->can_id = PROTOCOL_AUTOMATIC_STEERING;
            ptrFrameMsg->can_byte_array[0] = 10;
            break;
        case RECEIVED_PROTOCOL_ERROR:
            if (!ProtocolErrorTypeQueue.empty())
            {
                ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
                ptrFrameMsg->can_id = PROTOCOL_ERROR_FRAMES;
                ptrFrameMsg->can_byte_array[ERROR_TYPE_BYTE] =
                ProtocolErrorTypeQueue.front();
                ProtocolErrorTypeQueue.pop();
                ptrFrameMsg->can_byte_array[ERROR_SPECIFIC_BYTE] =
                ProtocolErrorSpecificQueue.front();
                ProtocolErrorSpecificQueue.pop();
            }
            else
            {
                ptrFrameMsg->can_id = 0;
            }
            break;
        case RECEIVED_MANUALSTEERING_ERROR:
            if (!ManualErrorTypeQueue.empty())
            {
                ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
                ptrFrameMsg->can_id = PROTOCOL_ERROR_FRAMES;
                ptrFrameMsg->can_byte_array[ERROR_TYPE_BYTE] =
                ManualErrorTypeQueue.front();
                ManualErrorTypeQueue.pop();
                ptrFrameMsg->can_byte_array[ERROR_SPECIFIC_BYTE] =
                ManualErrorSpecificQueue.front();
                ManualErrorSpecificQueue.pop();
            }
            else
            {
                ptrFrameMsg->can_id = 0;
            }
            break;
        case RECEIVED_CONFIGURATION:
             ptrFrameMsg->can_id = PROTOCOL_CONFIGURATION_FRAME;
             if (bReceivedCompleteConfig)
                ptrFrameMsg->can_byte_array[CONFIG_TYPE_BYTE] = 100;
             break;
        default:
            ptrFrameMsg->can_id = 0;
            break;
    }
}

float MobileAppHandler::fGetManualRadius()
{
    return dRadius;
}

float MobileAppHandler::fGetManualLinearVelo()
{
    return dManualLinearVelo;
}

void MobileAppHandler::vSetSimulationAppFrame(
    const uint32_t uRecFrameID,
    const boost::array<unsigned char,
     8> _chRecBytes)
{
    iCommandId = uRecFrameID;
    msg_byte_array[0] = _chRecBytes[0];
    msg_byte_array[1] = _chRecBytes[1];
    msg_byte_array[2] = _chRecBytes[2];
    msg_byte_array[3] = _chRecBytes[3];
    msg_byte_array[4] = _chRecBytes[4];
    msg_byte_array[5] = _chRecBytes[5];
    msg_byte_array[6] = _chRecBytes[6];
    msg_byte_array[7] = _chRecBytes[7];
    bFreshData = true;
}

// Method used to register and inform state machine about incoming msg
void MobileAppHandler::vSetSimulationMoveFrame(
    float fFrLin,
    float fFrAng,
    float fBkLin,
    float fBkAng)
{
    if (!bFreshData)
    {
        if (fFrLin != 0 && fFrAng != 0 && fBkLin != 0 && fBkAng != 0)
        {
            iCommandId = 53;
            fFrontLinear = fFrLin;
            fFrontAngular = fFrAng;
            fBackLinear = fBkLin;
            fBackAngular = fBkAng;
            bFreshData = true;
        }
    }
}

void MobileAppHandler::vSetChosenTask(std::string strTaskName)
{
    strChosenTask = strTaskName;
}

void MobileAppHandler::vCreateSimStateMachineAppFrame(
    agribot_msgs::AgribotMobileAppFrame* ptrFrame)
{
    if (bFreshData)
    {
        ptrFrame->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
        ptrFrame->can_id = iCommandId;
        ptrFrame->can_byte_array[0] = msg_byte_array[0];
        ptrFrame->can_byte_array[1] = msg_byte_array[1];
        ptrFrame->can_byte_array[2] = msg_byte_array[2];
        ptrFrame->can_byte_array[3] = msg_byte_array[3];
        ptrFrame->can_byte_array[4] = msg_byte_array[4];
        ptrFrame->can_byte_array[5] = msg_byte_array[5];
        ptrFrame->can_byte_array[6] = msg_byte_array[6];
        ptrFrame->can_byte_array[7] = msg_byte_array[7];

        bFreshData = false;
    }
}


std::string MobileAppHandler::strGetTaskName()
{
    std::string strTemp = strChosenTask;
    strChosenTask = "";
    return strTemp;
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

bool MobileAppHandler::bDataPendingSimulation()
{
    return bFreshData;
}
