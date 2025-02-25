#pragma once

#include <gafro/algebra.hpp>
#include <gafro/robot/System.hpp>

namespace gafro
{

    template <class T>
    class ForwardKinematics
    {
      public:
        ForwardKinematics(const System<T> &system, const Eigen::VectorX<T> &joint_positions, const Motor<T> &base_pose = Motor<T>());

        const Motor<T> &getJointPose(const std::string &joint_name) const;

        const Motor<T> &getLinkPose(const std::string &link_name) const;

        const std::map<std::string, Motor<T>> &getLinkPoses() const;

      private:
        void processLink(const Eigen::VectorX<T> &joint_positions,  //
                         const gafro::Link<double> *link,           //
                         const gafro::Motor<double> &parent_pose);

      private:
        std::map<std::string, Motor<T>> joint_poses_;

        std::map<std::string, Motor<T>> link_poses_;
    };

}  // namespace gafro

#include <gafro/robot/algorithm/ForwardKinematics.hxx>