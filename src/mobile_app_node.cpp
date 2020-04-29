#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/AgribotManualControl.h>
#include <std_msgs/Int32.h>
#include <industrial_msgs/RobotMode.h>
#include <geometry_msgs/Twist.h>
#include <hal_mobile_app/MobileAppHandler.h>
#include <std_msgs/String.h>
#include <sstream>

#define USED_CAN "can1"
#define HEADER_STRING_MOB_APP_FRAME "mobile_app_commands_link"

MobileAppHandler CommHandlerControl(USED_CAN);

void vAppCommandsSimulationEventsCallback(
  const agribot_msgs::AgribotMobileAppFrame::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetSimulationAppFrame(ptrMsg->can_id,
                                            ptrMsg->can_byte_array);
}

void vAppTaskChoosedEventCallback(
  const std_msgs::String::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetChosenTask(ptrMsg->data);
}

void vFillTheMessageObject(
  agribot_msgs::AgribotMobileAppFrame* ptrFrameMsg,
   uint uCanID, uint8_t* u8MsgBytes)
{
  ptrFrameMsg->header.frame_id = HEADER_STRING_MOB_APP_FRAME;
  ptrFrameMsg->header.stamp = ros::Time::now();
  ptrFrameMsg->can_id = uCanID;
  for (int i = 0; i < 8; i++)
    ptrFrameMsg->can_byte_array[i] = u8MsgBytes[i];
}

int main(int argc, char **argv)
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
    ros::Publisher StateMachinePub = nh.advertise
    <agribot_msgs::AgribotMobileAppFrame>
    ("agribot/mobile_app_communication", 1000);
    // Publish cmd_vel_manual from application commands
    ros::Publisher PublishManualSteering = nh.advertise
    <agribot_msgs::AgribotManualControl>
    ("/cmd_vel_manual", 1);
    // Topic used to communicate change of state to the state machine
    ros::Publisher StateMachineModePub = nh.advertise
    <industrial_msgs::RobotMode>
    ("/agribot/state_machine/mode", 1);
    // Topic used to communicate change of state to the state machine
    ros::Publisher StateMachineErrorPub = nh.advertise
    <std_msgs::Int32>
    ("/agribot/state_machine/error", 1);
    // Topic used to read the path name from task config file
    ros::Subscriber AppTaskChoosedEventsSub = nh.subscribe
    ("/agribot/chosen_task_name", 1,
    vAppTaskChoosedEventCallback);


    if (bSimulation == true )
    {
      ros::Subscriber AppCommandsEventsSub = nh.subscribe
      ("/agribot/app_commands_events_simulation", 1,
      vAppCommandsSimulationEventsCallback);
    }

    ros::Rate loop_rate(10);

    agribot_msgs::AgribotMobileAppFrame FrameMsg;
    agribot_msgs::AgribotManualControl ManualSteeringMsg;
    industrial_msgs::RobotMode RobMode;
    std_msgs::Int32 ErrorIndicator;
    uint8_t u8MsgBytes[8];
    uint uCanID = 0;
    uint8_t u8Status = 0x00;

    while (ros::ok())
    {
      name_tmp = CommHandlerControl.strGetTaskName();
      if (name_tmp != "")
      {
        nh.setParam("/agribot/chosen_task_name", name_tmp);
      }

      // If used in simulation
      if (bSimulation == true )
      {
          CommHandlerControl.update();

          if (CommHandlerControl.bDataPendingSimulation())
          {
              CommHandlerControl.vCreateSimStateMachineAppFrame
                                                    (&uCanID, u8MsgBytes);
              vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
              StateMachinePub.publish(FrameMsg);
          }
      }
      else  // If used on the machine with CAN
      {
          ErrorIndicator.data = 0;
          CommHandlerControl.vReceiveCanFrames();
          u8Status = CommHandlerControl.u8ParseCanFrames();
          if ((u8Status&RECEIVED_MANUAL_STEERING) ==
              RECEIVED_MANUAL_STEERING)
          {
              ManualSteeringMsg.linear_velocity =
                CommHandlerControl.fGetManualLinearVelo();
              ManualSteeringMsg.circle_radius =
                CommHandlerControl.fGetManualRadius();
              PublishManualSteering.publish(ManualSteeringMsg);

              CommHandlerControl.vAssemblyStateMachineFrame(
                                &uCanID, u8MsgBytes, RECEIVED_MANUAL_STEERING);
              vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
              StateMachinePub.publish(FrameMsg);
          }
          if ((u8Status&RECEIVED_MANUAL_MODE_COMF) ==
              RECEIVED_MANUAL_MODE_COMF)
          {
              RobMode.val = RobMode.MANUAL;
              StateMachineModePub.publish(RobMode);
              CommHandlerControl.vAssemblyStateMachineFrame(
                              &uCanID, u8MsgBytes, RECEIVED_MANUAL_MODE_COMF);
              vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
              StateMachinePub.publish(FrameMsg);
          }
          if ((u8Status&RECEIVED_AUTO_STEERING) ==
              RECEIVED_AUTO_STEERING)
          {
              RobMode.val = RobMode.AUTO;
              CommHandlerControl.vAssemblyStateMachineFrame(
                                  &uCanID, u8MsgBytes, RECEIVED_AUTO_STEERING);
              vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
              StateMachinePub.publish(FrameMsg);
          }
          if ((u8Status&RECEIVED_PROTOCOL_ERROR) ==
              RECEIVED_PROTOCOL_ERROR)
          {
              ErrorIndicator.data = 1;
              StateMachineErrorPub.publish(ErrorIndicator);
              while (true)
              {
                CommHandlerControl.vAssemblyStateMachineFrame(
                                &uCanID, u8MsgBytes, RECEIVED_PROTOCOL_ERROR);
                if (uCanID == 0)
                  break;
                else
                {
                  vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
                  StateMachinePub.publish(FrameMsg);
                }
              }
          }
          if ((u8Status&RECEIVED_MANUALSTEERING_ERROR) ==
               RECEIVED_MANUALSTEERING_ERROR)
          {
              ErrorIndicator.data = 2;
              StateMachineErrorPub.publish(ErrorIndicator);
              while (true)
              {
                CommHandlerControl.vAssemblyStateMachineFrame(
                          &uCanID, u8MsgBytes, RECEIVED_MANUALSTEERING_ERROR);
                if (uCanID == 0)
                  break;
                else
                {
                  vFillTheMessageObject(&FrameMsg, uCanID, u8MsgBytes);
                  StateMachinePub.publish(FrameMsg);
                }
              }
          }
          if (ErrorIndicator.data == 0)
          {
              // Send no error to the state machine
              StateMachineErrorPub.publish(ErrorIndicator);
          }
          if ((u8Status&RECEIVED_CONFIGURATION) == RECEIVED_CONFIGURATION)
          {
            while (true)
              {
                CommHandlerControl.vAssemblyStateMachineFrame(
                                  &uCanID, u8MsgBytes, RECEIVED_CONFIGURATION);
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
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
