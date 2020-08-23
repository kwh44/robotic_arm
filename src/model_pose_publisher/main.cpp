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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace gazebo {

    class ModelPose : public ModelPlugin {
        physics::ModelPtr model;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
        rclcpp::node ros_node;
        rclcpp::TimerBase::SharedPtr timer;
    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            model = _parent;
            ros_node = rclcpp::Node("pose_ros_publisher");
            publisher = ros_node.create_publisher<std_msgs::msg::Float64MultiArray>("/beer_model_pose", 10);
            timer = std::chrono_literals::create_wall_timer(
                    500ms, std::bind(&ModelPose::timer_callback, this));

        }

        void timer_callback() {
            std_msgs::Float64MultiArray message;
            message.resize(3);
            auto pose = this->model->WorldPose();
            ignition::math::Vector3 v(0, 0, 0);
            v = pose.Pos();
            double vec[3];
            for (size_t i = 0; i < 3; ++i) vec[i] = v[i];
            message.data = vec;
            publisher->publish(message);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPose)
}