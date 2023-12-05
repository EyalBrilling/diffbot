#ifndef ODOM_TO_GPS_PLUGIN_HH
#define ODOM_TO_GPS_PLUGIN_HH

#include <ros/ros.h>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

// Callback queue for subscriber
#include <ros/callback_queue.h>

namespace gazebo
{

   class OdomToGpsPlugin : public WorldPlugin
   {
      public:
      OdomToGpsPlugin();
      virtual ~OdomToGpsPlugin();

     protected:
     void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
     void onUpdate();
     
     private:
        ros::NodeHandle* node_handle_;
        ros::Subscriber odom_subscriber_;
        ros::Publisher gps_publisher_;
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        
    // Pointer to the model
        physics::ModelPtr model;
     
     // gazebo connection
     event::ConnectionPtr updateConnection;

    std::string namespace_;

    // Save messages 
     sensor_msgs::NavSatFix gps_info;
     nav_msgs::Odometry odom_info;

    // Queue
    ros::CallbackQueue queue_;

   };

}

#endif