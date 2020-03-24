#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/AgribotManualControl.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>
#include <hal_mobile_app/MobileAppHandler.h>

#define USED_CAN "can1"


MobileAppHandler CommHandlerControl(USED_CAN);

void vAppCommandsSimulationEventsCallback(const agribot_msgs::AgribotMobileAppFrame::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetSimulationAppFrame(ptrMsg->can_id, ptrMsg->can_byte_array);
}

void vAppTaskChoosedEventCallback(const std_msgs::String::ConstPtr& ptrMsg)
{
  CommHandlerControl.vSetChosenTask(ptrMsg->data);
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
    //Use this topic to communicate application events with the robot state machine
    ros::Publisher StateMachinePub = nh.advertise<agribot_msgs::AgribotMobileAppFrame>("agribot/mobile_app_communication", 1000);
    //Publish cmd_vel_manual from application commands
    ros::Publisher PublishManualSteering = nh.advertise<agribot_msgs::AgribotManualControl>("/cmd_vel_manual",1);
    //Topic used to read the path name from task config file
    ros::Subscriber AppTaskChoosedEventsSub = nh.subscribe("/agribot/chosen_task_name", 1, vAppTaskChoosedEventCallback);

    if (bSimulation == true )
    {
      ros::Subscriber AppCommandsEventsSub = nh.subscribe("/agribot/app_commands_events_simulation", 1, vAppCommandsSimulationEventsCallback);
    }

    ros::Rate loop_rate(10);

    agribot_msgs::AgribotMobileAppFrame FrameMsg;
    agribot_msgs::AgribotManualControl ManualSteeringMsg;
    
    uint8_t u8Status=0x00;

    while (ros::ok())
    {
      name_tmp = CommHandlerControl.strGetTaskName();
      if (name_tmp != "")
      {
        nh.setParam("/agribot/chosen_task_name", name_tmp);
      }

      //If used in simulation
      if (bSimulation == true )
      {
          CommHandlerControl.update();

          if (CommHandlerControl.bDataPendingSimulation())
          {

              CommHandlerControl.vCreateSimStateMachineAppFrame(&FrameMsg);

              ROS_INFO("%s", FrameMsg.header.frame_id.c_str());

              StateMachinePub.publish(FrameMsg);

          }
      } 
      else
      //If used on the machine with CAN
      {
          CommHandlerControl.vReceiveCanFrames();
          u8Status=CommHandlerControl.u8ParseCanFrames();
          if((u8Status&RECEIVED_MANUAL_STEERING) == RECEIVED_MANUAL_STEERING)
          {
              ManualSteeringMsg.linear_velocity = CommHandlerControl.fGetManualLinearVelo();
              ManualSteeringMsg.circle_radius = CommHandlerControl.fGetManualRadius();
              PublishManualSteering.publish(ManualSteeringMsg);

              CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_MANUAL_STEERING);
              StateMachinePub.publish(FrameMsg);
          }
          if((u8Status&RECEIVED_MANUAL_MODE_COMF) == RECEIVED_MANUAL_MODE_COMF)
          {
              CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_MANUAL_MODE_COMF);
              StateMachinePub.publish(FrameMsg);
          }
          if((u8Status&RECEIVED_AUTO_STEERING) == RECEIVED_AUTO_STEERING)
          {
              CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_AUTO_STEERING);
              StateMachinePub.publish(FrameMsg);
          }
          if((u8Status&RECEIVED_PROTOCOL_ERROR) == RECEIVED_PROTOCOL_ERROR)
          {
              while(true)
              {
                CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_PROTOCOL_ERROR);
                if(FrameMsg.can_id == 0)
                  break;
                else
                  StateMachinePub.publish(FrameMsg);
              }
          }
          if((u8Status&RECEIVED_MANUALSTEERING_ERROR) == RECEIVED_MANUALSTEERING_ERROR)
          {
              while(true)
              {
                CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_MANUALSTEERING_ERROR);
                if(FrameMsg.can_id == 0)
                  break;
                else
                  StateMachinePub.publish(FrameMsg);
              }
          }
          if((u8Status&RECEIVED_CONFIGURATION) == RECEIVED_CONFIGURATION)
          {
            CommHandlerControl.vAssemblyStateMachineFrame(&FrameMsg, RECEIVED_CONFIGURATION);
            StateMachinePub.publish(FrameMsg);
          }
      }
      ros::spinOnce();

      loop_rate.sleep();
    }

  return 0;
}