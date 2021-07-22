/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <agribot_msgs/AgribotAppErrorFlag.h>
#include <agribot_msgs/AgribotManualControl.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/AgribotTool.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <geometry_msgs/Twist.h>
#include <hal_mobile_app/MobileAppConstants.h>
#include <hal_mobile_app/MobileAppHandler.h>
#include <industrial_msgs/RobotMode.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <sstream>

#define HEADER_STRING_MOB_APP_FRAME "mobile_app_commands_link"

MobileAppHandler CommHandlerControl;

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

tool_manager::TplConfig tplMsgToStruct(const agribot_msgs::AgribotTool& msg)
{
  tool_manager::TplConfig feedback;
  feedback.pto_rpm = msg.pto_rpm;
  feedback.red_valve_flow = msg.red_valve_flow;
  feedback.green_valve_flow = msg.green_valve_flow;
  feedback.blue_valve_flow = msg.blue_valve_flow;
  feedback.tpl_angle = msg.tpl_angle;
  feedback.tpl_height = msg.tpl_height;
  return feedback;
}

void frontToolFeedbackCallback(const agribot_msgs::AgribotTool& msg)
{
  std::vector<std::vector<std::uint8_t>> frames_data;
  frames_data = CommHandlerControl.prepareTplFeedbackFrames(tplMsgToStruct(msg));
  CommHandlerControl.sendTplFeedbackFrames(TplConfig::SelectedTpl::front, frames_data);
}

void backToolFeedbackCallback(const agribot_msgs::AgribotTool& msg)
{
  std::vector<std::vector<std::uint8_t>> frames_data;
  frames_data = CommHandlerControl.prepareTplFeedbackFrames(tplMsgToStruct(msg));
  CommHandlerControl.sendTplFeedbackFrames(TplConfig::SelectedTpl::back, frames_data);
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
                     agribot_msgs::ToolManagerScan& service,  // NOLINT(runtime/references)
                     uint8_t sel_ant)
{
  service.request.requested_state = sel_ant;
  bool srv_ret = client.call(service);
  if (srv_ret && (service.response.result == service.response.SUCCESS))
  {
    CommHandlerControl.vSendRfidTagsIds(sel_ant, castMsgVectorToStdVec(service.response.ids));
  }
}

agribot_msgs::AgribotTool tplConfigToToolMsg(const tool_manager::TplConfig& config)
{
  agribot_msgs::AgribotTool tool_msg;
  tool_msg.tpl_angle = config.tpl_angle;
  tool_msg.tpl_height = config.tpl_height;
  tool_msg.green_valve_flow = config.green_valve_flow;
  tool_msg.red_valve_flow = config.red_valve_flow;
  tool_msg.blue_valve_flow = config.blue_valve_flow;
  tool_msg.pto_rpm = config.pto_rpm;
  return tool_msg;
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

  std::string selected_can;
  try
  {
    selected_can = ros_common_functions::getRosParam<std::string>("/hal_mobile_app/can_bus_name");
  }
  catch (const std::string& ex)
  {
    ROS_ERROR_STREAM("[HalMobileApp] " + ex);
    ros::shutdown();
  }
  std::unique_ptr<Can> can_handler = std::make_unique<Can>();
  can_handler->iInitialize(selected_can.c_str());
  CommHandlerControl.initializeCanHandler(std::move(can_handler));
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
  // Topic for feedback from front TPL:
  ros::Subscriber tpl_front_sub = nh.subscribe("/agribot/encoder/tool/front", 1, frontToolFeedbackCallback);
  // Topic for feedback from back TPL:
  ros::Subscriber tpl_back_sub = nh.subscribe("/agribot/encoder/tool/back", 1, backToolFeedbackCallback);
  // Topic used to read the path name from task config file
  ros::Publisher StateMachineTaskPub = nh.advertise<std_msgs::String>("/agribot/state_machine/task", 1);
  // The client of scan tags service
  ros::ServiceClient rfid_tags_scanner_client =
      nh.serviceClient<agribot_msgs::ToolManagerScan>("agribot/tool_management/scan");
  agribot_msgs::ToolManagerScan rfid_tags_scanner_srv;
  // Tpl config publishers
  ros::Publisher front_tpl_config_publisher =
      nh.advertise<agribot_msgs::AgribotTool>("/agribot/control/tool/front/manual", 1);
  ros::Publisher back_tpl_config_publisher =
      nh.advertise<agribot_msgs::AgribotTool>("/agribot/control/tool/back/manual", 1);
  ros::ServiceClient change_delta_geometry_front_client =
      nh.serviceClient<agribot_msgs::ChangeDeltasGeometry>("/agribot/cart/caterpillar/front/change_deltas_geometry");
  ros::ServiceClient change_delta_geometry_back_client =
      nh.serviceClient<agribot_msgs::ChangeDeltasGeometry>("/agribot/cart/caterpillar/back/change_deltas_geometry");

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
  uint8_t u8MsgBytes[8];
  uint uCanID = 0;
  uint16_t u16Status = 0x0000;

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
      if ((u16Status & RECEIVED_AUTO_STEERING) == RECEIVED_AUTO_STEERING)
      {
        RobMode.val = RobMode.AUTO;
        StateMachineModePub.publish(RobMode);
        CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_AUTO_STEERING);
        vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
        StateMachinePub.publish(FrameMsg);
      }
      if ((u16Status & RECEIVED_PROTOCOL_ERROR) == RECEIVED_PROTOCOL_ERROR)
      {
        ErrorIndicator.error_flag = ErrorIndicator.APP_PROTOCOL_ERROR;
        StateMachineErrorPub.publish(ErrorIndicator);
        while (true)
        {
          CommHandlerControl.vAssemblyStateMachineFrame(&uCanID, u8MsgBytes, RECEIVED_PROTOCOL_ERROR);
          if (uCanID == 0)
          {
            break;
          }
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
          {
            break;
          }
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
          {
            break;
          }
          else
          {
            vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
            StateMachinePub.publish(FrameMsg);
          }
        }
      }
      if ((u16Status & RECEIVED_CHANGE_DELTA_GEOMETRY_FRONT) == RECEIVED_CHANGE_DELTA_GEOMETRY_FRONT)
      {
        agribot_msgs::ChangeDeltasGeometry service;
        if (CommHandlerControl.bGetChangeDeltaGeometryUpFront())
        {
          service.request.cart_delta_command =
              agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY;
        }
        else
        {
          service.request.cart_delta_command = agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY;
        }

        if (change_delta_geometry_front_client.call(service))
        {
          CommHandlerControl.sendDeltasGeometryResponseFrameFront(service.response.result);
        }
      }
      if ((u16Status & RECEIVED_CHANGE_DELTA_GEOMETRY_BACK) == RECEIVED_CHANGE_DELTA_GEOMETRY_BACK)
      {
        agribot_msgs::ChangeDeltasGeometry service;
        if (CommHandlerControl.bGetChangeDeltaGeometryUpBack())
        {
          service.request.cart_delta_command =
              agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY;
        }
        else
        {
          service.request.cart_delta_command = agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY;
        }

        if (change_delta_geometry_back_client.call(service))
        {
          CommHandlerControl.sendDeltasGeometryResponseFrameBack(service.response.result);
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
    // Tool control publish:
    front_tpl_config_publisher.publish(
        tplConfigToToolMsg(CommHandlerControl.getTplConfig(TplConfig::SelectedTpl::front)));
    back_tpl_config_publisher.publish(
        tplConfigToToolMsg(CommHandlerControl.getTplConfig(TplConfig::SelectedTpl::back)));
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
