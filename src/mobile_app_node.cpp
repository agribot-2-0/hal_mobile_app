/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/AgribotManualControl.h>
#include <agribot_msgs/AgribotAppErrorFlag.h>
#include <industrial_msgs/RobotMode.h>
#include <geometry_msgs/Twist.h>
#include <hal_mobile_app/MobileAppHandler.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sstream>

#define HEADER_STRING_MOB_APP_FRAME "mobile_app_commands_link"

const char cestrUsedCAN[] = "can1";

MobileAppHandler CommHandlerControl(cestrUsedCAN);

void synchro_callback(const ros::TimerEvent& event)
{
  CommHandlerControl.vSetTriggerForController();
}

void vAppCommandsSimulationEventsCallback(const agribot_msgs::AgribotMobileAppFrame::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetSimulationAppFrame(ptrMsg->can_id, ptrMsg->can_byte_array);
}

void vAppTaskChoosedEventCallback(const std_msgs::String::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetChosenTask(ptrMsg->data);
}

void vFillTheMessageObject(agribot_msgs::AgribotMobileAppFrame* ptrFrameMsg, uint uCanID, uint8_t* u8MsgBytes)
{
  ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
  ptrFrameMsg->header.stamp = ros::Time::now();
  ptrFrameMsg->can_id = uCanID;
  for (int i = 0; i < 8; i++)
  {
    ptrFrameMsg->can_byte_array[i] = u8MsgBytes[i];
  }
}

auto castMsgVectorToStdVec(std::vector<agribot_msgs::Tag>& msg_vec)  // NOLINT(runtime/references)
{
  std::vector<std::vector<uint8_t>> std_vec;
  for (agribot_msgs::Tag id : msg_vec)
  {
    std_vec.push_back(std::vector<uint8_t>(id.epc));
  }
  return std_vec;
}

void callRfidScanSrv(ros::ServiceClient client,
                     agribot_msgs::ToolManagerScan& service,
                     uint8_t sel_ant)
{
  service.request.requested_state = sel_ant;
  bool srv_ret = client.call(service);
  if (srv_ret && (service.response.result == service.response.SUCCESS))
  {
    CommHandlerControl.vSendRfidTagsIds(sel_ant,
                                        castMsgVectorToStdVec(service.response.ids));
  }
}

int main(int argc, char** argv)
{
  std::string name_tmp = "";
  ros::init(argc, argv, "hal_mobile_app");
  ros::NodeHandle nh;
  bool bSimParam;
  bool bSimulation = false;
  if (nh.getParam("/agribot_commander/sim", bSimParam))
  {
    if (bSimParam)
    {
      bSimulation = true;
    }
    else
    {
      bSimulation = false;
    }
  }
  // Use this topic to communicate application events
  // with the robot state machine
  ros::Publisher StateMachinePub =
      nh.advertise<agribot_msgs::AgribotMobileAppFrame>("agribot/mobile_app_communication", 1000);
  // Publish cmd_vel_manual from application commands
  ros::Publisher PublishManualSteering = nh.advertise<agribot_msgs::AgribotManualControl>("/cmd_vel_manual", 1);
  // Topic used to communicate change of state to the state machine
  ros::Publisher StateMachineModePub = nh.advertise<industrial_msgs::RobotMode>("/agribot/state_machine/mode", 1);
  // Topic used to communicate change of state to the state machine
  ros::Publisher StateMachineErrorPub =
      nh.advertise<agribot_msgs::AgribotAppErrorFlag>("/agribot/state_machine/error", 2);
  // Topic used to read the path name from task config file
  ros::Subscriber AppTaskChoosedEventsSub = nh.subscribe("/agribot/chosen_task_name", 1, vAppTaskChoosedEventCallback);
  // Topic used to read the path name from task config file
  ros::Publisher StateMachineTaskPub = nh.advertise<std_msgs::String>("/agribot/state_machine/task", 1);
  // The client of scan tags service
  ros::ServiceClient rfid_tags_scanner_client =
    nh.serviceClient<agribot_msgs::ToolManagerScan>("agribot/tool_management/scan");
  agribot_msgs::ToolManagerScan rfid_tags_scanner_srv;

  if (bSimulation == true)
  {
    ros::Subscriber AppCommandsEventsSub =
        nh.subscribe("/agribot/app_commands_events_simulation", 1, vAppCommandsSimulationEventsCallback);
  }

  // synchronization timer to achieve +-3Hz signal for kinematic control
  ros::Timer timer = nh.createTimer(ros::Duration(0.3), synchro_callback);

  ros::Rate loop_rate(10);

  agribot_msgs::AgribotMobileAppFrame FrameMsg;
  agribot_msgs::AgribotManualControl ManualSteeringMsg;
  industrial_msgs::RobotMode RobMode;
  agribot_msgs::AgribotAppErrorFlag ErrorIndicator;
  // workaround feature
  // for instant sending of the default configuration
  // without the option to choose
  std_msgs::String task_name;
  bool latching_configuration = false;
  uint8_t u8MsgBytes[8];
  uint uCanID = 0;
  uint16_t u16Status = 0x00;

  while (ros::ok())
  {
    name_tmp = CommHandlerControl.strGetTaskName();
    if (name_tmp != "")
    {
      nh.setParam("/agribot/chosen_task_name", name_tmp);
    }

    // If used in simulation
    if (bSimulation == true)
    {
      CommHandlerControl.update();

      if (CommHandlerControl.bDataPendingSimulation())
      {
        CommHandlerControl.vCreateSimStateMachineAppFrame(&uCanID, u8MsgBytes);
        vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
        StateMachinePub.publish(FrameMsg);
      }
    }
    else  // If used on the machine with CAN
    {
      ErrorIndicator.error_flag = ErrorIndicator.NO_APP_ERRORS;
      CommHandlerControl.vReceiveCanFrames();
      u16Status = CommHandlerControl.u16ParseCanFrames();
      if (CommHandlerControl.vReadTriggerForController())
      {
        if (CommHandlerControl.bCheckFreshArcData())
        {
          ManualSteeringMsg.linear_velocity = CommHandlerControl.fGetManualLinearVelo();
          ManualSteeringMsg.circle_radius = CommHandlerControl.fGetManualRadius();
          PublishManualSteering.publish(ManualSteeringMsg);
        }
        else
        {
          ManualSteeringMsg.linear_velocity = 0;
          ManualSteeringMsg.circle_radius = 0;
          PublishManualSteering.publish(ManualSteeringMsg);
        }
        CommHandlerControl.vResetArcReadTriggers();
        CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_MANUAL_STEERING);
        vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
        StateMachinePub.publish(FrameMsg);
      }
      if ((u16Status & RECEIVED_MANUAL_MODE_COMF) == RECEIVED_MANUAL_MODE_COMF)
      {
        RobMode.val = RobMode.MANUAL;
        StateMachineModePub.publish(RobMode);
        CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_MANUAL_MODE_COMF);
        vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
        StateMachinePub.publish(FrameMsg);
      }
      if ((u16Status & RECEIVED_AUTO_STEERING) == RECEIVED_AUTO_STEERING || latching_configuration)
      {
        if (!latching_configuration)
        {
          task_name.data = "test_task_drzewna";
          StateMachineTaskPub.publish(task_name);
          latching_configuration = true;
        }
        else
        {
          RobMode.val = RobMode.AUTO;
          StateMachineModePub.publish(RobMode);
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_AUTO_STEERING);
          vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
          StateMachinePub.publish(FrameMsg);
          latching_configuration = false;
        }
      }
      if ((u16Status & RECEIVED_PROTOCOL_ERROR) == RECEIVED_PROTOCOL_ERROR)
      {
        ErrorIndicator.error_flag = ErrorIndicator.APP_PROTOCOL_ERROR;
        StateMachineErrorPub.publish(ErrorIndicator);
        while (true)
        {
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_PROTOCOL_ERROR);
          if (uCanID == 0)
            break;
          else
          {
            vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
            StateMachinePub.publish(FrameMsg);
          }
        }
      }
      if ((u16Status & RECEIVED_MANUALSTEERING_ERROR) == RECEIVED_MANUALSTEERING_ERROR)
      {
        ErrorIndicator.error_flag = ErrorIndicator.APP_MANUAL_ERROR;
        StateMachineErrorPub.publish(ErrorIndicator);
        while (true)
        {
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_MANUALSTEERING_ERROR);
          if (uCanID == 0)
            break;
          else
          {
            vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
            StateMachinePub.publish(FrameMsg);
          }
        }
      }
      if (ErrorIndicator.error_flag == ErrorIndicator.NO_APP_ERRORS)
      {
        // Send no error to the state machine
        StateMachineErrorPub.publish(ErrorIndicator);
      }
      if ((u16Status & RECEIVED_CONFIGURATION) == RECEIVED_CONFIGURATION)
      {
        while (true)
        {
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_CONFIGURATION);
          if (uCanID == 0)
            break;
          else
          {
            vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
            StateMachinePub.publish(FrameMsg);
          }
        }
      }
      if ((u16Status & RECEIVED_ENG_ERROR_START) == RECEIVED_ENG_ERROR_START)
      {
        CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_ENG_ERROR_START);
        vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
        StateMachinePub.publish(FrameMsg);
      }
      if ((u16Status & RECEIVED_ENG_ERROR) == RECEIVED_ENG_ERROR)
      {
        while (true)
        {
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_ENG_ERROR);
          if (uCanID == 0)
            break;
          else
          {
            vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
            StateMachinePub.publish(FrameMsg);
          }
        }
      }
    }
    // Tool manager RFID tools scanning
    if (CommHandlerControl.bScanFrontTriggered())
    {
      callRfidScanSrv(rfid_tags_scanner_client, rfid_tags_scanner_srv, rfid_tags_scanner_srv.request.SCAN_FRONT);
    }
    if (CommHandlerControl.bScanBackTriggered())
    {
      callRfidScanSrv(rfid_tags_scanner_client, rfid_tags_scanner_srv, rfid_tags_scanner_srv.request.SCAN_BACK);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
