// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/algorithm/ForwardKinematics.hpp>

namespace gafro
{

    template <class T>
    ForwardKinematics<T>::ForwardKinematics(const System<T> &system, const Eigen::VectorX<T> &joint_positions, const Motor<T> &base_pose)
    {
        processLink(joint_positions, system.getLinks().front().get(), base_pose);
    }

    template <class T>
    const Motor<T> &ForwardKinematics<T>::getJointPose(const std::string &joint_name) const
    {
        return joint_poses_.at(joint_name);
    }

    template <class T>
    const Motor<T> &ForwardKinematics<T>::getLinkPose(const std::string &link_name) const
    {
        return link_poses_.at(link_name);
    }

    template <class T>
    const std::map<std::string, Motor<T>> &ForwardKinematics<T>::getLinkPoses() const
    {
        return link_poses_;
    }

    template <class T>
    void ForwardKinematics<T>::processLink(const Eigen::VectorX<T> &joint_positions,  //
                                           const gafro::Link<T>    *link,             //
                                           const gafro::Motor<T>   &parent_pose)
    {
        link_poses_.emplace(std::make_pair(link->getName(), parent_pose));

        for (const auto *child_joint : link->getChildJoints())
        {
            Motor<T> child_joint_pose;

            if (child_joint->getChildLink())
            {
                if (child_joint->isActuated())
                {
                    T joint_position;

                    if (child_joint->getIndex() < joint_positions.rows())
                    {
                        joint_position = joint_positions[child_joint->getIndex()];
                    }
                    else
                    {
                        joint_position = 0.0;
                    }

                    child_joint_pose = parent_pose * child_joint->getMotor(joint_position);
                }
                else
                {
                    child_joint_pose = parent_pose * child_joint->getMotor(0.0);
                }

                joint_poses_.emplace(std::make_pair(child_joint->getName(), child_joint_pose));

                processLink(joint_positions, child_joint->getChildLink(), child_joint_pose);
            }
        }
    }
}  // namespace gafro
