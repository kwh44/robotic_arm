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

namespace gazebo {
    class ModelPose : public ModelPlugin {

        physics::ModelPtr model;
        transport::PublisherPtr model_pose_pub_;
        transport::NodePtr node_handle_;

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            model = _parent;
            std::bind(&ModelPose::OnUpdate, this);
            node_handle_ = transport::NodePtr(new transport::Node());
            node_handle_->Init("mara2");
            model_pose_pub_ = node_handle_->Advertise<gazebo::msgs::Pose>("~/" + model->GetName() + "model_pose");

        }

        void OnUpdate() {
            auto pose = this->model->WorldPose();
            gazebo::msgs::Pose msg;
            gazebo::msgs::Set(&msg, pose);
            model_pose_pub_->Publish(msg);
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPose)
}