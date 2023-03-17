#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh> 

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "ros/callback_queue.h"

#include "std_msgs/Int32.h"
#include <thread>
#include <memory>
#include <string>
namespace gazebo{

class WorldPluginTutorial : public WorldPlugin
{
private: static std::string getNodeName()
    {
        return "listener_node";
    }

private: static std::string getTopicName()
    {
        return "/ros_gazebo_test/test/counter";
    }

private: void init_node(std::string name)
    {
        int argc = 0;
        char ** argv = NULL;
        ros::init(argc,argv,name);
        this->rosNode = std::unique_ptr<ros::NodeHandle> (new ros::NodeHandle());
        ROS_INFO("Node from new thread");
    }

/// \brief ROS helper function that process messages
private: void QueueThread()
    {
      static const double timeout = 0.1;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

private: void testCallback(const std_msgs::Int32::ConstPtr & msg)
    {
            ROS_INFO("Contador from plugin: %d", msg->data);
    }

private: void onUpdate()
    {
        return;
    }

public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world = _world;
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldPluginTutorial::onUpdate, this));
    
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      init_node(getNodeName());
      return;
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle(getNodeName()));  

    // Message subscriber
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>(
                                    getTopicName(),
                                    1,
                                    boost::bind(&WorldPluginTutorial::testCallback, this, _1),
                                    ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);
    
    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&WorldPluginTutorial::QueueThread, this));

    ROS_INFO("Plugin loaded successfully");

    return;
    }

    private: physics::WorldPtr world;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

}
