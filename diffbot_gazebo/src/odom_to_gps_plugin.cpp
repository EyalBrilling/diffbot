#include <gazebo/physics/physics.hh>
#include <diffbot_gazebo/odom_to_gps_plugin.h>

namespace gazebo {

void OdomToGpsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

    ROS_WARN("Loading odomToGps plugin to gazebo world");
    ROS_INFO("Received Odometry message");

    // load parameters
    if (!_sdf->HasElement("robotNamespace"))
        namespace_.clear();
    else
        namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
      

       this->updateConnection = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OdomToGpsPlugin::onUpdate, this));

      // Topic names
      std::string gps_topic = "/robot_location";
      std::string odom_topic ="/diffbot/mobile_base_controller/odom";

     node_handle_ = new ros::NodeHandle("odomToGps");


    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<nav_msgs::Odometry>(odom_topic, 1,
          boost::bind(&OdomToGpsPlugin::odomCallback, this, _1),
          ros::VoidPtr(),&queue_);

    odom_subscriber_ = node_handle_->subscribe(so);
    gps_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic, 1);

}

void OdomToGpsPlugin::onUpdate(){

}

void OdomToGpsPlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {

   ROS_INFO("Received Odometry message. Pose: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

}

OdomToGpsPlugin::OdomToGpsPlugin() : WorldPlugin(){
}

OdomToGpsPlugin::~OdomToGpsPlugin(){
}

GZ_REGISTER_WORLD_PLUGIN(OdomToGpsPlugin)

}