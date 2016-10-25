/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
/*
 * Adapted as a ROS gazebo plugin for car-like vehicles using
 * Ackermann steering.
 *
 * Copyright (C) 2011, Nicu Stiurca, Jack O'Quin
 */
#ifndef ACKERMANN_PLUGIN_H
#define ACKERMANN_PLUGIN_H

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <stdio.h>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class Entity;

class AckermannPlugin : public ModelPlugin
{

public:
  AckermannPlugin();
  virtual ~AckermannPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  void Reset();
  virtual void Update();


private:
  void write_position_data();
  void publish_odometry();
  void GetPositionCmd();
  
  physics::WorldPtr world;
  physics::LinkPtr link;
  physics::JointPtr joints[4];
  

  double wheelDistance;
  double CarLength;
  double wheelDiam;
  double driveTorque;
  double steerTorque;
  double wheelSpeed[2];
  double steerAngle;

  // Simulation time of the last update
  common::Time prevUpdateTime;
  
  
  bool enableMotors;
  double odomPose[3];
  double odomVel[3];


  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string robotNamespace;

  std::string topicName;

  std::string linkName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // Ackermann stuff
  void carDriveCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  float speed_;
  float angle_;
  bool alive_;
  
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
  
};

}

#endif // ACKERMANN_PLUGIN_H
