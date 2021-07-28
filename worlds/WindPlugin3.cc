
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
 * obtenido de la pagina 
 * https://github.com/RigidWing/gazebo_plugins/blob/master/include/WindPlugin.hh
*/

#include <functional>
#include <thread>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/sensors/Noise.hh"

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

#include "WindPlugin3.hh"


/// \brief Private class for WindPlugin
class gazebo::WindPlugin3Private
{
  /// \brief World pointer.
  public: physics::WorldPtr world;

  /// \brief Connection to World Update events.
  public: event::ConnectionPtr updateConnection;

  /// \brief Time for wind to rise
  public: double characteristicTimeForWindRise = 1;

  /// \brief Wind amplitude
  public: double magnitudeSinAmplitudePercent = 0;

  /// \brief Wind period
  public: double magnitudeSinPeriod = 1;

  /// \brief Time for wind to change direction.
  public: double characteristicTimeForWindOrientationChange = 1;

  /// \brief Orientation amplitude
  public: double orientationSinAmplitude = 0;

  /// \brief Orientation period
  public: double orientationSinPeriod = 1;

  /// \brief period over characteristicTimeForWindRise
  public: double kMag = 0;

  /// \brief period over characteristicTimeForWindOrientationChange
  public: double kDir = 0;

  /// \brief Mean of the magnitude
  public: double magnitudeMean = 0;

  /// \brief Mean of the direction
  public: double directionMean = 0;

  /// \brief Noise added to magnitude
  public: sensors::NoisePtr noiseMagnitude;

  /// \brief Noise added to direction
  public: sensors::NoisePtr noiseDirection;

  /// \brief Noise added to Z axis
  public: sensors::NoisePtr noiseVertical;

  /// \brief Time for wind to rise
  public: double characteristicTimeForWindRiseVertical = 1;

  /// \brief period over characteristicTimeForWindRiseVertical
  public: double kMagVertical = 0;

  /// \brief Mean of the magnitude
  public: double magnitudeMeanVertical = 0;

  /// \brief The scaling factor to approximate wind as force on a mass.
  public: double C_D = 0;

  private: ros::Publisher _conversionPub1;

};


using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(WindPlugin3)

/////////////////////////////////////////////////
WindPlugin3::WindPlugin3()
    : dataPtr(new WindPlugin3Private)
{

}




/////////////////////////////////////////////////
void WindPlugin3::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "WindPlugin3 world pointer is NULL");
  this->dataPtr->world = _world;

  physics::Wind &wind = this->dataPtr->world->Wind();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "my_ros_node",ros::init_options::NoSigintHandler);
    ROS_INFO("iniciando nodo");
  }
  // Create ROS node.
  

  this->rosNode.reset(new ros::NodeHandle( "my_ros_node" ));
  this->_state_pub = _nh.advertise<nav_msgs::Odometry>("/wind_state", 5, true);

  this->_master1_pub = _nh.advertise<nav_msgs::Odometry>("/drone1/realilinkpose", 5, true);
  this->_servant1_pub = _nh.advertise<nav_msgs::Odometry>("/drone2/realilinkpose", 5, true);
  this->_servant2_pub = _nh.advertise<nav_msgs::Odometry>("/drone3/realilinkpose", 5, true);

  this->_master4_pub = _nh.advertise<nav_msgs::Odometry>("/drone4/realilinkpose", 5, true);
  this->_servant5_pub = _nh.advertise<nav_msgs::Odometry>("/drone5/realilinkpose", 5, true);
  this->_servant6_pub = _nh.advertise<nav_msgs::Odometry>("/drone6/realilinkpose", 5, true);

  this->pruebitaPub = _nh.advertise<nav_msgs::Odometry>("prueba_mensaje_odometria",10);
  //this->_conversionPub1 = _nh.advertise<nav_msgs::Odometry>("/drone1/dato_de_otro_drone", 5, true);

  //no funciona aún una simulacion de varios vehiculos controlados mediante ROS 
  //this->_dosauno_sub = n.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10, conversion_cb);
  //this->_unoados_sub = n.subscribe<nav_msgs::Odometry>("/drone2/mavros/global_position/local", 10, conversion_cb);
  //this->_unoatres_sub = n.subscribe<nav_msgs::Odometry>("/drone3/mavros/global_position/local", 10, conversion_cb);
  //this->_tresauno_sub = n.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10, conversion_cb);
  //this->_tresados_sub = n.subscribe<nav_msgs::Odometry>("/drone2/mavros/global_position/local", 10, conversion_cb);
  //this->_dosatres_sub = n.subscribe<nav_msgs::Odometry>("/drone3/mavros/global_position/local", 10, conversion_cb);




  this->rosPub = this->rosNode->advertise<std_msgs::Float64>("my_ros_topic",1);
  ROS_INFO("NodeHandle");



  if (_sdf->HasElement("horizontal"))
  {
    sdf::ElementPtr sdfHoriz = _sdf->GetElement("horizontal");

    if (sdfHoriz->HasElement("magnitude"))
    {
      sdf::ElementPtr sdfMag = sdfHoriz->GetElement("magnitude");

      if (sdfMag->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindRise =
          sdfMag->Get<double>("time_for_rise");
      }

      if (sdfMag->HasElement("sin"))
      {
        sdf::ElementPtr sdfMagSin = sdfMag->GetElement("sin");

        if (sdfMagSin->HasElement("amplitude_percent"))
        {
          this->dataPtr->magnitudeSinAmplitudePercent =
            sdfMagSin->Get<double>("amplitude_percent");
        }

        if (sdfMagSin->HasElement("period"))
        {
          this->dataPtr->magnitudeSinPeriod = sdfMagSin->Get<double>("period");
        }
      }

      if (sdfMag->HasElement("noise"))
      {
        this->dataPtr->noiseMagnitude = sensors::NoiseFactory::NewNoiseModel(
              sdfMag->GetElement("noise"));
      }
    }

    if (sdfHoriz->HasElement("direction"))
    {
      sdf::ElementPtr sdfDir = sdfHoriz->GetElement("direction");

      if (sdfDir->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindOrientationChange =
          sdfDir->Get<double>("time_for_rise");
      }

      if (sdfDir->HasElement("sin"))
      {
        sdf::ElementPtr sdfDirSin = sdfDir->GetElement("sin");

        if (sdfDirSin->HasElement("amplitude"))
        {
          this->dataPtr->orientationSinAmplitude =
            sdfDirSin->Get<double>("amplitude");
        }

        if (sdfDirSin->HasElement("period"))
        {
          this->dataPtr->orientationSinPeriod =
            sdfDirSin->Get<double>("period");
        }
      }

      if (sdfDir->HasElement("noise"))
      {
        this->dataPtr->noiseDirection = sensors::NoiseFactory::NewNoiseModel(
            sdfDir->GetElement("noise"));
      }
    }
  }

  if (_sdf->HasElement("vertical"))
  {
    sdf::ElementPtr sdfVert = _sdf->GetElement("vertical");

    if (sdfVert->HasElement("time_for_rise"))
    {
      this->dataPtr->characteristicTimeForWindRiseVertical =
        sdfVert->Get<double>("time_for_rise");
    }

    if (sdfVert->HasElement("noise"))
    {
      this->dataPtr->noiseVertical = sensors::NoiseFactory::NewNoiseModel(
            sdfVert->GetElement("noise"));
    }
  }

  if (_sdf->HasElement("C_D"))
  {
    sdf::ElementPtr sdfForceApprox =
      _sdf->GetElement("C_D");

    this->dataPtr->C_D =
        sdfForceApprox->Get<double>();
  }

  // If the C_D is very small don't update.
  // It doesn't make sense to be negative, that would be negative wind drag.
  if (std::fabs(this->dataPtr->C_D) < 1e-6)
  {
    gzerr << "Please set C_D to a value "
          << "greater than 0" << std::endl;
    return;
  }

  double period = this->dataPtr->world->Physics()->GetMaxStepSize();

  this->dataPtr->kMag = period / this->dataPtr->characteristicTimeForWindRise;
  this->dataPtr->kMagVertical =period / this->dataPtr->characteristicTimeForWindRiseVertical;
  this->dataPtr->kDir =period / this->dataPtr->characteristicTimeForWindOrientationChange;

  //procesos mas importantes 

  wind.SetLinearVelFunc(std::bind(&WindPlugin3::LinearVel, this,std::placeholders::_1, std::placeholders::_2));

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WindPlugin3::OnUpdate, this));
  //Fin de procesos mas importantes 
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindPlugin3::LinearVel(const physics::Wind *_wind, const physics::Entity * /*_entity*/)
{
  // Compute magnitude
  this->dataPtr->magnitudeMean = (1. - this->dataPtr->kMag) * this->dataPtr->magnitudeMean + this->dataPtr->kMag *
      std::sqrt(_wind->LinearVel().X() * _wind->LinearVel().X() +_wind->LinearVel().Y() * _wind->LinearVel().Y());
  double magnitude = this->dataPtr->magnitudeMean;

  // Compute magnitude
  this->dataPtr->magnitudeMeanVertical = (1. - this->dataPtr->kMagVertical) *this->dataPtr->magnitudeMeanVertical +
      this->dataPtr->kMagVertical * _wind->LinearVel().Z();

  magnitude += this->dataPtr->magnitudeSinAmplitudePercent *this->dataPtr->magnitudeMean *
    std::sin(2 * M_PI * this->dataPtr->world->SimTime().Double() /this->dataPtr->magnitudeSinPeriod);

  if (this->dataPtr->noiseMagnitude)
  {
    magnitude = this->dataPtr->noiseMagnitude->Apply(magnitude);
  }

  // Compute horizontal direction
  //
  double direction = IGN_RTOD(atan2(_wind->LinearVel().Y(),
                                   _wind->LinearVel().X()));

  this->dataPtr->directionMean = (1.0 - this->dataPtr->kDir) *
      this->dataPtr->directionMean + this->dataPtr->kDir * direction;

  direction = this->dataPtr->directionMean;

  direction += this->dataPtr->orientationSinAmplitude *
      std::sin(2 * M_PI * this->dataPtr->world->SimTime().Double() /
        this->dataPtr->orientationSinPeriod);

  if (this->dataPtr->noiseDirection)
    direction = this->dataPtr->noiseDirection->Apply(direction);

  // Apply wind velocity
  ignition::math::Vector3d speedo;
  speedo.X(magnitude * std::cos(IGN_DTOR(direction)));
  speedo.Y(magnitude * std::sin(IGN_DTOR(direction)));

  if (this->dataPtr->noiseVertical)
  {
    speedo.Z(this->dataPtr->noiseVertical->Apply(
        this->dataPtr->magnitudeMeanVertical));
  }
  else
  {
    speedo.Z(this->dataPtr->magnitudeMeanVertical);
  }
  //ROS_WARN("LinearSpeed = [%f,%f,%f] ",speedo.X(),speedo.Y(),speedo.Z());



  return speedo;
  //this->rosPub = this->rosNode->advertise<ignition::math::v4::Vector3d>("my_ros_topic",1);
  //this->rosPub = this->rosNode->advertise<std_msgs::Float64>("my_ros_topic",1);
  //std_msgs::Float64 msg;
  //msg.data = 15.3;
  //this->rosPub.publish(msg);
  //std_msgs::Float64 msg;
  //msg.data = 15.3;
  //this->_state_pub.publish(msg);


}

int i=0;

/////////////////////////////////////////////////
void WindPlugin3::OnUpdate()
{
  // Update loop for using the force on mass approximation
  // This is not recommended. Please use the LiftDragPlugin instead.

  // Get all the models
  physics::Model_V models = this->dataPtr->world->Models();


  // Process each model.
  for (auto const &model : models)
  {
    // Get all the links
    physics::Link_V links = model->GetLinks();
    std::string nombre=model->GetName();

    // Process each link.
    for (auto const &link : links)
    {
      // Skip links for which the wind is disabled
      if (!link->WindMode())
        continue;


      std_msgs::Float64 msg;
      //ignition::math::Vector3d linear;
      std::string ElNombre=link->GetName();


      // Add wind velocity as a force to the body
      //link->AddRelativeForce(link->GetInertial()->Mass() *this->dataPtr->forceApproximationScalingFactor *0.1081*1.225*0.5*
      //    (link->RelativeWindLinearVel() - link->RelativeLinearVel())*(link->RelativeWindLinearVel() - link->RelativeLinearVel()));
      ignition::math::Vector3d diff;
      diff=(link->RelativeWindLinearVel() - link->RelativeLinearVel());
      //ignition::math::Vector3d mag_diff;
      double mag_diff=std::sqrt(diff.X()*diff.X()+diff.Y()*diff.Y()+diff.Z()*diff.Z());
      double area=0;
      double area2;
      double C_Df=0;
      double psi=0.635120953;
      if (ElNombre=="iris_demo::iris::base_link")
      {
        //area=2*(0.0875+0.35*0.23);
        area = 0.02965*4;
        //double D_esfera=0.88;
        double D_esfera=0.157871944*2;
        area2= M_PI*D_esfera*D_esfera/4;

        //se suma un pequeño valor al reynolds para no tener divisiones por 0
        double Re=1.225*mag_diff*D_esfera/0.0000174+0.000001;
        //https://pages.mtu.edu/~fmorriso/DataCorrelationForSphereDrag2016.pdf
        //double C_D1=24/Re+0.52*Re/(1+std::pow(Re,1.52))+0.411*std::pow(Re/(2.63*std::pow(10,5)),-7.94)/(1+std::pow(Re/(2.63*std::pow(10,5)),-8))+0.25*(Re/std::pow(10,6))/(1+Re/std::pow(10,6));
        //C_D=21.12/Re+6.3/std::sqrt(Re)+0.25;
        //C_Df=this->dataPtr->C_D1;
        double C_D1=24/Re*(1+std::exp(2.3288-6.4581*psi+2.4486*psi*psi))*std::pow(Re,(0.0964+0.5565*psi))+Re*std::exp(4.905-13.8944*psi+18.4222*psi*psi-10.2599*psi*psi*psi)/(Re+std::exp(1.4681+12.2584*psi-20.7322*psi*psi+15.8855*psi*psi*psi)); 
        C_Df=C_D1;
      }

      if (ElNombre=="iris::camera")
      {
        area=0.00327;
        C_Df=0.1;

      }

      if (ElNombre=="iris::hokuyo_link")
      {
        area=0.06;
        C_Df=0.1;

      }
    
      ignition::math::Vector3d force;
      force=C_Df *area2*1.225*0.5*(diff)*(mag_diff);
      link->AddRelativeForce(force);
      //now_lin_vel = this->link->RelativeLinearVel();
      //ROS_WARN("LinearSpeed = [%f,%f,%f] ",this->now_lin_vel.X(), this->now_lin_vel.Y(), this->now_lin_vel.Z());

      //odom_.pose.pose.position.x  = link->RelativeWindLinearVel().X();
      //odom_.pose.pose.position.y  = link->RelativeWindLinearVel().Y();
      //odom_.pose.pose.position.z  = link->RelativeWindLinearVel().Z();

      odom_.pose.pose.position.x  = force.X();
      odom_.pose.pose.position.y  = force.Y();
      odom_.pose.pose.position.z  = force.Z();

      
      odom_.twist.twist.linear.x = link->WorldWindLinearVel().X();
      odom_.twist.twist.linear.y = link->WorldWindLinearVel().Y();
      odom_.header.frame_id = "odom_";

      odom_.child_frame_id = ElNombre;

      ignition::math::Pose3d posicionreal;
      posicionreal=link->WorldPose();
      ignition::math::Vector3d arreglo=posicionreal.Pos();
      float last_yaw = posicionreal.Rot().Yaw();
      float w = posicionreal.Rot().W();
      //ignition::math::Quaternion quat=posicionreal.Rot();
      //ignition::math::Quaternion rot = link->WorldPose()->Rot();
      realpos_.pose.pose.position.x  = arreglo.X();
      realpos_.pose.pose.position.y  = arreglo.Y();
      realpos_.pose.pose.position.z  = arreglo.Z();
      realpos_.pose.pose.orientation.x=posicionreal.Rot().X();
      realpos_.pose.pose.orientation.y=posicionreal.Rot().Y();
      realpos_.pose.pose.orientation.z=posicionreal.Rot().Z();
      realpos_.pose.pose.orientation.w=posicionreal.Rot().W();
      realpos_.child_frame_id = ElNombre;
      realpos_.header.frame_id = nombre;

      //ROS_WARN("ReLativeLinearSpeed = [%f,%f] ",linear.X(), linear.Y());
      if (ElNombre=="iris_demo::iris::base_link"||ElNombre=="drone1::iris_demo::iris::base_link")
      {
        
        if (i%100==0)
        {
          //ROS_WARN("i = [%u] ",i);
          this->_state_pub.publish(odom_);
          this->pruebitaPub.publish(odom_);


          if (nombre=="drone1")
          {
            this->_master1_pub.publish(realpos_);
          }
          else if (nombre=="drone2")
          {
            this->_servant1_pub.publish(realpos_);
          }
          else if (nombre=="drone3")
          {
            this->_servant2_pub.publish(realpos_);            
          }  
          else if (nombre=="drone4")
          {
            this->_master4_pub.publish(realpos_);
          }
          else if (nombre=="drone5")
          {
            this->_servant5_pub.publish(realpos_);
          }
          else if (nombre=="drone6")
          {
            this->_servant6_pub.publish(realpos_);
          }  

        }

        i=i+1;

      }




    }
  }
}



