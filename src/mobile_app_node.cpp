#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>
#include <hal_mobile_app/MobileAppHandler.h>

#define USED_CAN "can1"


MobileAppHandler CommHandlerControl(USED_CAN);

void app_commands_eventsCallback(const agribot_msgs::AgribotMobileAppFrame::ConstPtr& msg)
{
  CommHandlerControl.setSimulationAppFrame(msg->can_id, msg->can_byte_array);
}

void app_task_choosed_eventCallback(const std_msgs::String::ConstPtr& msg)
{
  CommHandlerControl.setChosenTask(msg->data);
}

int main(int argc, char **argv)
{
    bool simulation = false;
  std::string name_tmp = "";
  ros::init(argc, argv, "hal_mobile_app");
  ros::NodeHandle nh;

  ros::Publisher state_machine_pub = nh.advertise<agribot_msgs::AgribotMobileAppFrame>("agribot/mobile_app_communication", 1000);
  ros::Publisher PublishManualSteering = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  ros::Subscriber app_task_choosed_events_sub = nh.subscribe("/agribot/chosen_task_name", 1, app_task_choosed_eventCallback);

  if (simulation == true )
  {
    ros::Subscriber app_commands_events_sub = nh.subscribe("/agribot/app_commands_events", 1, app_commands_eventsCallback);
  }

  ros::Rate loop_rate(10);

  agribot_msgs::AgribotMobileAppFrame frame_msg;
  geometry_msgs::Twist ManualSteeringMsg;
  
  uint8_t u8Status=0x00;

  while (ros::ok())
  {
    name_tmp = CommHandlerControl.strgetTaskName();
    if (name_tmp != "")
    {
      nh.setParam("/agribot/chosen_task_name", name_tmp);
    }

    //If used in simulation
    if (simulation == true )
    {
        CommHandlerControl.update();

        if (CommHandlerControl.dataPendingSimulation())
        {

            CommHandlerControl.createStateMachineAppFrame(&frame_msg);

            ROS_INFO("%s", frame_msg.header.frame_id.c_str());

            state_machine_pub.publish(frame_msg);

        }
    } 
    else
    //If used on the machine with CAN
    {
        CommHandlerControl.vReceiveCanFrames();
        u8Status=CommHandlerControl.u8ParseCanFrames();
        if((u8Status&RECEIVED_MANUAL_STEERING) == RECEIVED_MANUAL_STEERING)
        {
            ManualSteeringMsg.linear.x = CommHandlerControl.fGetManualLinearVelo();
            ManualSteeringMsg.linear.y = 0;
            ManualSteeringMsg.linear.z = 0;
            ManualSteeringMsg.angular.x = 0;
            ManualSteeringMsg.angular.y = 0;
            ManualSteeringMsg.angular.z = CommHandlerControl.fGetManualAngularVelo();
            PublishManualSteering.publish(ManualSteeringMsg);
            frame_msg.header.stamp = ros::Time::now();
            frame_msg.header.frame_id = "mobile_app_commands_link";
            frame_msg.can_id = 53;
            frame_msg.can_byte_array[0] = 10;
            state_machine_pub.publish(frame_msg);
        }
        if((u8Status&RECEIVED_MANUAL_MODE_COMF) == RECEIVED_MANUAL_MODE_COMF)
        {
            frame_msg.header.stamp = ros::Time::now();
            frame_msg.header.frame_id = "mobile_app_commands_link";
            frame_msg.can_id = 10;
            frame_msg.can_byte_array[0] = 10;
            state_machine_pub.publish(frame_msg);
        }
        if((u8Status&RECEIVED_AUTO_STEERING) == RECEIVED_AUTO_STEERING)
        {
            frame_msg.header.stamp = ros::Time::now();
            frame_msg.header.frame_id = "mobile_app_commands_link";
            frame_msg.can_id = 30;
            frame_msg.can_byte_array[0] = 10;
            state_machine_pub.publish(frame_msg);
        }
        if((u8Status&RECEIVED_AUTO_STEERING) == RECEIVED_AUTO_STEERING)
        {
            frame_msg.header.stamp = ros::Time::now();
            frame_msg.header.frame_id = "mobile_app_commands_link";
            frame_msg.can_id = 30;
            frame_msg.can_byte_array[0] = 10;
            state_machine_pub.publish(frame_msg);
        }
    }
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}