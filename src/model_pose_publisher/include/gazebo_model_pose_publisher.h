//
// Created by kwh44 on 8/26/20.
//

#ifndef GAZEBO_PLUGINS_GAZEBO_MODEL_POSE_PUBLISHER_H
#define GAZEBO_PLUGINS_GAZEBO_MODEL_POSE_PUBLISHER_H

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
    class GazeboRosEntityStatePublisherPrivate;


    class GazeboRosEntityStatePublisher : public gazebo::ModelPlugin
    {
    public:
        /// Constructor
        GazeboRosEntityStatePublisher();

        /// Destructor
        ~GazeboRosEntityStatePublisher();

    protected:
        // Documentation inherited
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:
        /// Callback to be called at every simulation iteration.
        /// Private data pointer
        std::unique_ptr<GazeboRosEntityStatePublisherPrivate> impl_;
    };
}  // namespace gazebo_plugins

#endif //GAZEBO_PLUGINS_GAZEBO_MODEL_POSE_PUBLISHER_H
