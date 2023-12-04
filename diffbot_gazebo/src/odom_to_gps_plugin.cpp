#include <gazebo/physics/physics.hh>
#include <diffbot_gazebo/odom_to_gps_plugin.h>

namespace gazebo {

void OdomToGpsPlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf ){
    // Store the pointer to the model
      this->model = _parent;
 
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&amp;OdomToGpsPlugin::onUpdate, this));
       
      this->old_secs =ros::Time::now().toSec();
      
      std::string gps_topic = "/robot_location";
      std::string odom_topic ="/diffbot/mobile_base_controller/cmd_vel"

     this->node_handle_.reset(new ros::NodeHandle("odomtogps_rosnode"));

     odom_subscriber_ = this->node_handle_.subscribe(odom_topic, 1000, odomSubCallback);
}

void OdomToGpsPlugin::onUpdate(){

}

void odomSubCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

GazeboRosGps::GazeboRosGps(){
}

GazeboRosGps::~GazeboRosGps(){
}

GZ_REGISTER_MODEL_PLUGIN(OdomToGpsPlugin)

}