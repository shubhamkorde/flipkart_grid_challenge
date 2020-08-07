#ifndef _ARM_CONTROL_PLUGIN_HH_
#define _ARM_CONTROL_PLUGIN_HH_

//  Gazebo api
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
/// \brief A plugin to control a Velodyne sensor.
class ArmControlPlugin : public ModelPlugin
{
public:
    /// \brief Constructor
    ArmControlPlugin(){ }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Arm Control Plugin not loaded\n";
            return;
        }

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint1 = _model->GetJoint("right_sec_arm_to_right_prim_track");
        this->joint2 = _model->GetJoint("left_sec_arm_to_left_prim_track");

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(500, 0, 1);
        
        this->prevUpdateTime = this->model->GetWorld()->RealTime();

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/arm_state";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName,&ArmControlPlugin::OnMsg, this);

        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
        
        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/arm_state",
                                                                                    1,
                                                                                    boost::bind(&ArmControlPlugin::OnRosMsg, this, _1),
                                                                                    ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread = std::thread(std::bind(&ArmControlPlugin::QueueThread, this));
    } // Load

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    void SetPosition(const double &_angle)
    {
        /*
        // Set the joint's target velocity.
        common::Time currTime = this->model->GetWorld()->RealTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime = currTime;

        if ( stepTime > 0 ) {
            double m_cmd_force1 = this->pid.Update(this->joint1->Position(0) - _angle, stepTime);
            double m_cmd_force2 = this->pid.Update(this->joint2->Position(0) - _angle, stepTime);
            std::cout<<"cmd_force  :" << m_cmd_force1<<"\n";
            std::cout<<"error      :" << this->joint2->Position(0) - _angle<<"\n";
       //     this->joint->SetEffortLimit(0,100.0);
            this->joint1->SetForce(0, m_cmd_force1);
            this->joint2->SetForce(0, m_cmd_force2);
        }
        */
       this->joint1->SetUpperLimit(0,_angle);
       this->joint1->SetLowerLimit(0,_angle);
       this->joint2->SetUpperLimit(0,_angle);
       this->joint2->SetLowerLimit(0,_angle);

    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
        this->SetPosition(_msg->data);
    }

private:
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetPosition(_msg->x());
    }

    /// \brief ROS helper function that processes messages
    void QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    physics::JointPtr joint1;
    physics::JointPtr joint2;

    /// \brief A PID controller for the joint.
    common::PID pid;

    /// \brief A node used for transport
    transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr sub;

    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    common::Time prevUpdateTime;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ArmControlPlugin)
}
#endif // ifndef _ARM_CONTROL_PLUGIN_HH_