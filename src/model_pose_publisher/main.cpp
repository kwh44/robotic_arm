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


namespace gazebo {

    class ModelPose : public ModelPlugin {

    private:
        physics::ModelPtr model;
        rclcpp::Publisher<gazebo::msgs::Pose>::SharedPtr publisher;
        rclcpp::Node ros_node;
        rclcpp::TimerBase::SharedPtr timer;
    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            model = _parent;
            int argc = 0;
            char **argv = NULL;
            rclcpp::init(argc, argv);
            ros_node.reset(new rclcpp::Node("pose_ros_publisher"));
            publisher = ros_node.create_publisher<gazebo::msgs::Pose>("/beer_model_pose", 10);
            timer = create_wall_timer(
                    500ms, std::bind(&ModelPose::timer_callback, this));
            rclcpp::spin(ros_node);
        }

        void timer_callback() {
            auto pose = this->model->WorldPose();
            gazebo::msgs::Pose msg;
            gazebo::msgs::Set(&msg, pose);
            publisher->publish(msg);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPose)
}