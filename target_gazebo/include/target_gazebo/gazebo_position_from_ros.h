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
#ifndef _GAZEBO_POSITION_FROM_ROS_PLUGIN_HH_
#define _GAZEBO_POSITION_FROM_ROS_PLUGIN_HH_

#include <boost/bind.hpp>
#include <stdio.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "PointStamped.pb.h"

namespace gazebo
{

typedef const boost::shared_ptr<const gz_geometry_msgs::PointStamped> GzPointPtr;

/// \brief A template model plugin
class GAZEBO_VISIBLE GazeboPositionFromRosPlugin : public ModelPlugin
{
public:
    /// \brief Constructor.
    GazeboPositionFromRosPlugin();

    /// \brief Destructor.
    ~GazeboPositionFromRosPlugin();

    // Documentation Inherited. SDF is not needed here
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

    /// \brief Fetch the latest victim drone positon
    void PosCallback(GzPointPtr& victim_position);

protected:
    /// \brief subscribe to victim drone position 
    void CreatePubsAndSubs();

    /// \brief Callback for World Update events.
    void OnUpdate();

    /// \brief Connection to World Update events.
    event::ConnectionPtr update_connection_;

    /// \brief Pointer to world.
    physics::WorldPtr world_;

    /// \brief Pointer to model containing plugin.
    physics::ModelPtr model_;

private:
    transport::NodePtr node_handle_;
    transport::SubscriberPtr pos_sub_;

    ignition::math::Vector3d victim_pos_;
    bool update_pos_{false};
    bool pubs_and_subs_created_{false};
};
}
#endif // _GAZEBO_POSITION_FROM_ROS_PLUGIN_HH_
