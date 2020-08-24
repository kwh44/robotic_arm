#include <functional>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
/*

namespace gazebo {

    class ModelPose : public ModelPlugin {

        private: physics::ModelPtr model;
        private: std::unique_ptr<ros::NodeHandle> rosNode;
        /// \brief A ROS callbackqueue that helps process messages
        private: rclcpp::CallbackQueue rosQueue;
        /// \brief A ROS subscriber
        private: rclcpp::Publisher rosPub;
        /// \brief A ROS callbackqueue that helps process messages
        private: rclcpp::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;
    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr ) {
            model = _parent;
            int argc = 0;
            char **argv = NULL;
            rclcpp::init(argc, argv);
            ros_node.reset(new rclcpp::Node("pose_ros_publisher"));
            // Create a named topic, and subscribe to it.
            rclcpp::PublishOptions po =
                    rclcpp::publishOptions::create<gazebo::msgs::Pose>(
                            "/" + this->model->GetName() + "/pose",
                            1,
                            boost::bind(&ModelPose::OnRosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);
        }

        void timer_callback() {
            auto pose = this->model->WorldPose();
            gazebo::msgs::Pose msg;
            gazebo::msgs::Set(&msg, pose);
            publisher->publish(msg);
        }
            /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
            {
                this->SetVelocity(_msg->data);
            }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
            {
                static const double timeout = 0.01;
                while (this->rosNode->ok())
                {
                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPose)
}
*/
namespace gazebo {
    class ModelPose : public ModelPlugin {
        rclcpp::Node *rosnode_;
        rclcpp::Publisher<std_msgs::String> pub_;
        rclcpp::PubQueue<std_msgs::String>::Ptr pub_Queue;
        rclcpp::PubMultiQueue pmq;
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            if (!rclcpp::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                rclcpp::init(argc, argv, "TestSpace", ros::init_options::NoSigintHandler);
            }
            rosnode_ = new rclcpp::Node("TestSpace");
            pmq.startServiceThread();
            pub_Queue = pmq.addPub<std_msgs::String>();
            pub_ = rosnode_->advertise<std_msgs::String>("/beer/model_pose", 1);
            std_msgs::String msg;
            msg.data = "0.4404040404 0.2342345 0.234532245433243";
            this->pub_Queue->push(msg, pub_);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelPose);
}