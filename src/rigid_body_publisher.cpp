/* 
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <mocap_optitrack/rigid_body_publisher.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

namespace mocap_optitrack
{

namespace utilities
{
  geometry_msgs::TransformStamped getRosPose(RigidBody const& body, bool newCoordinates)
  {
    geometry_msgs::TransformStamped transformStampedMsg;
    if (newCoordinates)
    {
      // Motive 1.7+ coordinate system
      transformStampedMsg.transform.translation.x = -body.pose.position.x;
      transformStampedMsg.transform.translation.y = body.pose.position.z;
      transformStampedMsg.transform.translation.z = body.pose.position.y;
  
      transformStampedMsg.transform.rotation.x = -body.pose.orientation.x;
      transformStampedMsg.transform.rotation.y = body.pose.orientation.z;
      transformStampedMsg.transform.rotation.z = body.pose.orientation.y;
      transformStampedMsg.transform.rotation.w = body.pose.orientation.w;
    }
    else
    {
      // y & z axes are swapped in the Optitrack coordinate system
      transformStampedMsg.transform.translation.x = body.pose.position.x;
      transformStampedMsg.transform.translation.y = -body.pose.position.z;
      transformStampedMsg.transform.translation.z = body.pose.position.y;
  
      transformStampedMsg.transform.rotation.x = body.pose.orientation.x;
      transformStampedMsg.transform.rotation.y = -body.pose.orientation.z;
      transformStampedMsg.transform.rotation.z = body.pose.orientation.y;
      transformStampedMsg.transform.rotation.w = body.pose.orientation.w;
    }
    return transformStampedMsg;
  }   
}

RigidBodyPublisher::RigidBodyPublisher(ros::NodeHandle &nh, 
  Version const& natNetVersion,
  PublisherConfiguration const& config) :
    config(config)
{
  if (config.publishPose)
    posePublisher = nh.advertise<geometry_msgs::TransformStamped>(config.poseTopicName, 1000);

  if (config.publishPose2d)
    pose2dPublisher = nh.advertise<geometry_msgs::Pose2D>(config.pose2dTopicName, 1000);

  // Motive 1.7+ uses a new coordinate system
  useNewCoordinates = (natNetVersion >= Version("1.7"));
}

RigidBodyPublisher::~RigidBodyPublisher()
{

}

void RigidBodyPublisher::publish(ros::Time const& time, RigidBody const& body)
{
  // don't do anything if no new data was provided
  if (!body.hasValidData())
  {
    return;
  }

  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  geometry_msgs::TransformStamped pose = utilities::getRosPose(body, useNewCoordinates);
  pose.header.stamp = time;

  if (config.publishPose)
  {
    pose.header.frame_id = config.parentFrameId;
    pose.child_frame_id = config.childFrameId;
    posePublisher.publish(pose);
  }

  tf::Quaternion q(pose.transform.rotation.x,
                   pose.transform.rotation.y,
                   pose.transform.rotation.z,
                   pose.transform.rotation.w);

  // publish 2D pose
  if (config.publishPose2d)
  {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.transform.translation.x;
    pose2d.y = pose.transform.translation.y;
    pose2d.theta = tf::getYaw(q);
    pose2dPublisher.publish(pose2d);
  }

  if (config.publishTf)
  {
    // publish transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.transform.translation.x,
                                     pose.transform.translation.y,
                                     pose.transform.translation.z));

    // Handle different coordinate systems (Arena vs. rviz)
    transform.setRotation(q);
    tfPublisher.sendTransform(tf::StampedTransform(transform, 
      time, 
      config.parentFrameId, 
      config.childFrameId));
  }
}


RigidBodyPublishDispatcher::RigidBodyPublishDispatcher(
  ros::NodeHandle &nh, 
  Version const& natNetVersion,
  PublisherConfigurations const& configs)
{
  for (auto const& config : configs)
  {
    rigidBodyPublisherMap[config.rigidBodyId] = 
      RigidBodyPublisherPtr(new RigidBodyPublisher(nh, natNetVersion, config));
  }
}

void RigidBodyPublishDispatcher::publish(
  ros::Time const& time, 
  std::vector<RigidBody> const& rigidBodies)
{
  for (auto const& rigidBody : rigidBodies)
  {
    auto const& iter = rigidBodyPublisherMap.find(rigidBody.bodyId);

    if (iter != rigidBodyPublisherMap.end())
    {
      (*iter->second).publish(time, rigidBody);
    }
  }
}


} // namespace