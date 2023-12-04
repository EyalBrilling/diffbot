#ifndef ODOM_TO_GPS_PLUGIN_HH
#define ODOM_TO_GPS_PLUGIN_HH

#include <ros/ros.h>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>


namespace gazebo
{

   class OdomToGpsPlugin : public ModelPlugin
   {
      public:
      OdomToGpsPlugin();
      virtual ~OdomToGpsPlugin();

     protected:
     void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
     void onUpdate();
     
     private:
        ros::NodeHandle* node_handle_;
        ros::Subscriber odom_subscriber_;
        ros::Publisher gps_publisher_;

    // Pointer to the model
    private: physics::ModelPtr model;
 
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
     
    // Time Memory
    double old_secs;

    // Save messages 
     sensor_msgs::NavSatFix gps_info;
     nav_msgs::Odometry odom_info;


   };

}

#endif