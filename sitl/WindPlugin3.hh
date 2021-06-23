/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_
#include <memory>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>


#include <common.h>
#include <gazebo_gps_plugin.h>
#include <boost/algorithm/string.hpp>





#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "nav_msgs/Odometry.h"
namespace gazebo
{
  // Forward declaration
  class WindPlugin3Private;

  /// \brief A plugin that simulates a simple wind model.
  // The wind is described as a uniform worldwide model. So it is independant
  // from model position for simple computations. Its components are computed
  // separately:
  // - Horizontal amplitude:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value (noise amplitude is a factor of wind magnitude)
  //
  // - Horizontal direction:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value
  //
  // - Vertical amplitude:
  //      Noise proportionnal to wind magnitude.
  class GAZEBO_VISIBLE WindPlugin3 : public WorldPlugin
  {
    /// \brief Constructor.
    public: WindPlugin3();

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Get the global wind velocity.
    /// \param[in] _wind Reference to the wind.
    /// \param[in] _wind Pointer to an entity at which location the wind
    /// velocity is to be calculated.
    /// \return Wind's velocity at entity's location.
    public: ignition::math::Vector3d LinearVel(
            const physics::Wind *_wind,
            const physics::Entity *_entity);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<WindPlugin3Private> dataPtr;
    

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher rosPub;
    private: nav_msgs::Odometry odom_;
    private: nav_msgs::Odometry realpos_;

    private: ros::NodeHandle _nh;
    private: ros::Publisher _state_pub;
    private: ros::Publisher _realpos_pub;
    private: ros::Publisher _servant1_pub;
    private: ros::Publisher _servant2_pub;
    private: ros::Publisher _master4_pub;
    private: ros::Publisher _servant5_pub;
    private: ros::Publisher _servant6_pub;
    private: ros::Publisher _conversionPub1;
    private: ros::Publisher conversionPub2;
    private: ros::Publisher conversionPub3;
    private: std_msgs::String _light_msg;
    private: std::stringstream _ss;
    private: ros::Subscriber _state_sub;

    private: ros::Publisher pruebitaPub;


        
        
        
        
        

        


  };
}

#endif
