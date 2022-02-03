/*
 * Copyright 2021 Jonathan Becker, ASL, ETH Zurich, Switzerland
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
 *
 */

#include "target_gazebo/gazebo_position_from_ros.h"

#include "ConnectRosToGazeboTopic.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboPositionFromRosPlugin)

/////////////////////////////////////////////////
GazeboPositionFromRosPlugin::GazeboPositionFromRosPlugin()
{
    gzdbg<<"GazeboPositionFromRosPlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboPositionFromRosPlugin::~GazeboPositionFromRosPlugin()
{
    gzdbg<<"GazeboPositionFromRosPlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboPositionFromRosPlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr /*_sdf*/)
{
    gzdbg<<"loading...\n";

    GZ_ASSERT(_model, "TemplatePlugin _model pointer is NULL");
    this->model_ = _model;
    
    this->world_ = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "TemplatePlugin world pointer is NULL");
    
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();
    
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPositionFromRosPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void GazeboPositionFromRosPlugin::CreatePubsAndSubs(){
    // Create temporary "ConnectRosToGazeboTopic" publisher and message
    gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
    node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
            "~/connect_ros_to_gazebo_subtopic", 1);

    // Connect to ROS
    gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
    // currently topics specific to the target drone, but could be remapped to anything
    connect_ros_to_gazebo_topic_msg.set_ros_topic("/target_drone/pos");
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("/gazebo/target_drone/position");
    connect_ros_to_gazebo_topic_msg.set_msgtype(
            gz_std_msgs::ConnectRosToGazeboTopic::POINT);
    gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                                true);
    pos_sub_ = node_handle_->Subscribe("/gazebo/target_drone/position", &GazeboPositionFromRosPlugin::PosCallback, this);
}

/////////////////////////////////////////////////
void GazeboPositionFromRosPlugin::PosCallback(GzPointPtr& victim_position){
    victim_pos_[0] = victim_position->point().x();
    victim_pos_[1] = victim_position->point().y();
    victim_pos_[2] = victim_position->point().z();
    update_pos_ = true;
}

/////////////////////////////////////////////////
void GazeboPositionFromRosPlugin::OnUpdate()
{
   if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }

    ignition::math::Quaterniond victim_rot(0, 0, 0, 1);
    ignition::math::Pose3d victim_pose(victim_pos_, victim_rot);
    this->model_->SetWorldPose(victim_pose);
    this->model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    this->model_->SetGravityMode(false);
}


