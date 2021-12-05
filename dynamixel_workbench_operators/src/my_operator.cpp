/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "dynamixel_workbench_operators/my_operator.h"

JointOperator::JointOperator()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_loop_(false)
{
  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
  std::string yaml_file1 = node_handle_.param<std::string>("serving_trajectory_info", "");
  std::string yaml_file2 = node_handle_.param<std::string>("cleaning_trajectory_info", "");

  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
  serving_motion_msg_ = new trajectory_msgs::JointTrajectory;

  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);
  bool result1 = getTrajectoryInfo(yaml_file1, serving_motion_msg_);


  if (result1 == false)
  {
    ROS_ERROR("Please check YAML file");
    exit(0);
  }
  dynamixel_command_subscriber_ = node_handle_.subscribe("dynamixel_position_command", 1000, &JointOperator::CommandMsgCallback, this);
  joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
  move_command_server_ = node_handle_.advertiseService("execution", &JointOperator::moveCommandMsgCallback, this);

  is_loop_ = priv_node_handle_.param<bool>("is_loop", "false");
}

JointOperator::~JointOperator()
{
  delete jnt_tra_msg_;
}

void JointOperator::CommandMsgCallback(const d2c_robot_msgs::DynamixelCommand::ConstPtr& msg)
{
  std::vector<std::vector<double>> target_joint_position;
  float motion_command = msg -> motion;

  if (motion_command == 0.0)
  {
    joint_trajectory_pub_.publish(*jnt_tra_msg_);
    ROS_INFO("publish dynamixel control info : %f", motion_command);
  }

  else if (motion_command == 1.0)
  {
    joint_trajectory_pub_.publish(*serving_motion_msg_);
    ROS_INFO("publish dynamixel control info : %f", motion_command);
  }

  else if (motion_command == 2.0) 
  {
      target_joint_position.resize(5);
      for (int i = 0; i < 4; ++i)
      {
        target_joint_position[i].resize(4);
      }
    

      for(int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          target_joint_position[i][j] = msg->joint_position[i].positions.at(j);
        }
      }
    cleaning_motion_msg_ = new trajectory_msgs::JointTrajectory;
    bool result2 = getTrajectoryInfo2(target_joint_position, cleaning_motion_msg_);
    joint_trajectory_pub_.publish(*cleaning_motion_msg_);
    ROS_INFO("publish dynamixel control info : %f", motion_command);
  }


}

bool JointOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
  YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t index = 0; index < joint_size; index++)
  {
    std::string joint_name = joint["names"][index].as<std::string>();
    jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
  }

  YAML::Node motion = file["motion"];
  uint8_t motion_size = motion["names"].size();

  for (uint8_t index = 0; index < motion_size; index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;
    //jnt_tra_point.positions.resize(10);
    //jnt_tra_point.time_from_start.resize(10);
    std::string name = motion["names"][index].as<std::string>();
    YAML::Node motion_name = motion[name];
    for (uint8_t size = 0; size < joint_size; size++)
    {
      if (joint_size != motion_name["step"].size())
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion_name["step"][size].as<double>());

      ROS_INFO("motion_name : %s, step : %f", name.c_str(), motion_name["step"][size].as<double>());
    }

    if (motion_name["time_from_start"] == NULL)
    {
      ROS_ERROR("Please check time_from_start. It must be set time_from_start each step");
      return 0;
    }

    jnt_tra_point.time_from_start.fromSec(motion_name["time_from_start"].as<double>());

    ROS_INFO("time_from_start : %f", motion_name["time_from_start"].as<double>());

    jnt_tra_msg->points.push_back(jnt_tra_point);
  }

  return true;
}

bool JointOperator::getTrajectoryInfo2(std::vector<std::vector<double>> joint_position, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{

  std::vector<std::string> joint = {"joint1", "joint2", "joint3","joint4"};
  for (uint8_t index = 0; index < joint.size(); index++)
  {
    jnt_tra_msg->joint_names.push_back(joint[index]);
  }
  std::vector<double> p1 = joint_position[0];
  std::vector<double> p2 = joint_position[1];
  std::vector<double> p3 = joint_position[2];
  ROS_INFO("%f, %f, %f,%f",p1[0], p1[1],p1[2],p1[3]);
  ROS_INFO("%f, %f, %f,%f",p2[0], p2[1],p2[2],p2[3]);
  ROS_INFO("%f, %f, %f,%f",joint_position[2][0], p3[1],p3[2],p3[3]);
  std::vector<std::string> motion_name = {"motion1","motion2","motion3","motion4","motion5","motion6","motion7","motion8"};
  std::vector<std::vector<double>> motion = {p1,p2,p3,{0.0, 0.0, 0.0, 0.0},{0.538, -0.06348, -0.58844, 0.65272},{0.538, -0.231, -0.672, 0.905},{-0.5, -0.150858, -0.804671, 0.956325},{0.0, 0.0, 0.0, 0.0} };
  std::vector<double> time_from_start = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};

  for (uint8_t index = 0; index < motion_name.size(); index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    for (uint8_t size = 0; size < joint.size(); size++)
    {
      if (joint.size() != 4)
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion[index][size]);

      ROS_INFO("motion_name : %s, step : %f", motion_name[index].c_str(), motion[index][size]);
    }

    jnt_tra_point.time_from_start.fromSec(time_from_start[index]);

    ROS_INFO("time_from_start : %f", time_from_start[index]);

    jnt_tra_msg->points.push_back(jnt_tra_point);
  }

  return true;
}

bool JointOperator::moveCommandMsgCallback(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
  joint_trajectory_pub_.publish(*jnt_tra_msg_);

  res.success = true;
  res.message = "Success to publish joint trajectory";

  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "joint_operator");
  JointOperator joint_operator;
  
  ROS_INFO("For now, you can use publish joint trajectory msgs by triggering service(/execution)");

  if (joint_operator.isLoop())
  {
    while(1)
    {
      std_srvs::Trigger trig;
      joint_operator.moveCommandMsgCallback(trig.request, trig.response);
    }
  }

  ros::spin();

  return 0;
}
