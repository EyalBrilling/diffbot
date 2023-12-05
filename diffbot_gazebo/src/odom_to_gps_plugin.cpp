#include <gazebo/physics/physics.hh>
#include <diffbot_gazebo/odom_to_gps_plugin.h>

#define X_ODOM_TO_GPS 32.072734
#define Y_ODOM_TO_GPS 34.787465
   
namespace gazebo {

void OdomToGpsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

    ROS_WARN("Loading odomToGps plugin to gazebo world");

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

    this->callback_queue_thread_ = 
      boost::thread(boost::bind(&OdomToGpsPlugin::QueueThread, this));
      

}

void OdomToGpsPlugin::onUpdate(){
    gps_publisher_.publish(gps_info);
}

void OdomToGpsPlugin::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
   #ifdef DEBUG
   ROS_INFO("Received Odometry message. Pose: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
   #endif
   odom_info.pose.pose.position.x = msg->pose.pose.position.x;
   odom_info.pose.pose.position.y = msg->pose.pose.position.y;
   odom_info.pose.pose.position.z = msg->pose.pose.position.z;

   gps_info.latitude = odom_info.pose.pose.position.x + X_ODOM_TO_GPS;
   gps_info.longitude = odom_info.pose.pose.position.y + Y_ODOM_TO_GPS;

//    #ifdef DEBUG
   ROS_INFO("Resulted sensor_msgs::NavSatFix message:[%f, %f]", gps_info.latitude, gps_info.longitude);
//    #endif

}

void OdomToGpsPlugin::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && node_handle_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

OdomToGpsPlugin::OdomToGpsPlugin() : WorldPlugin(){
}

OdomToGpsPlugin::~OdomToGpsPlugin(){
    alive_ = false;
    queue_.clear();
    queue_.disable();
    node_handle_->shutdown();
    callback_queue_thread_.join();
}

GZ_REGISTER_WORLD_PLUGIN(OdomToGpsPlugin)

}