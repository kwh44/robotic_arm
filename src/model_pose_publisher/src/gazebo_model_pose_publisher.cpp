//
// Created by kwh44 on 8/26/20.
//

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "gazebo/msgs/msgs.hh"
#include <gazebo_plugins/gazebo_model_pose_publisher.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/Point.h>
#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
    class GazeboRosEntityStatePublisherPrivate
    {
    public:
        /// Callback to be called at every simulation iteration.
        /// \param[in] info Updated simulation info.
        void OnUpdate(const gazebo::common::UpdateInfo & info);

        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Joint state publisher.
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr entity_state_pub_;

        /// Period in seconds
        double update_period_;

        /// Keep last time an update was published
        gazebo::common::Time last_update_time_;

        /// Pointer to the update event connection.
        gazebo::event::ConnectionPtr update_connection_;

        gazebo::physics::ModelPtr model;
    };

    GazeboRosJointStatePublisher::GazeboRosJointStatePublisher()
            : impl_(std::make_unique<GazeboRosJointStatePublisherPrivate>())
    {
    }

    GazeboRosJointStatePublisher::~GazeboRosJointStatePublisher()
    {
    }

    void GazeboRosEntityStatePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        impl_->model = model;
        // ROS node
        impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
        // Get QoS profiles
        const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
        // Update rate
        double update_rate = 100.0;
        impl_->update_period_ = 1.0 / update_rate;
        impl_->last_update_time_ = model->GetWorld()->SimTime();
        // Entity state publisher
        impl_->entity_state_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::Pose>(
                "entity_state", qos.get_publisher_qos("entity_state", rclcpp::QoS(1000)));
        // Callback on every iteration
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboRosEntityStatePublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
    }

    void GazeboRosEntityStatePublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
    {
        auto pose = model->WorldPose();
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        entity_state_pub_->Publish(msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosEntityStatePublisher)
}  // namespace gazebo_plugins
