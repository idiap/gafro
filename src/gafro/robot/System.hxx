/*
    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <gafro/gafro_package_config.hpp>
//
#include <gafro/robot/FixedJoint.hxx>
#include <gafro/robot/Joint.hxx>
#include <gafro/robot/Link.hxx>
#include <gafro/robot/PrismaticJoint.hxx>
#include <gafro/robot/RevoluteJoint.hxx>
//
#include <gafro/physics/Twist.hxx>
#include <gafro/physics/Wrench.hxx>
//
#include <gafro/robot/System.hpp>
#include <gafro/robot/algorithm/InverseDynamics.hxx>

namespace gafro
{

    template <class T>
    System<T>::System()
    {}

    template <class T>
    System<T>::System(System &&other)
    {
        if (other.dof_ == 0)
            other.finalize();

        *this = std::move(other);
    }

    template <class T>
    System<T> &System<T>::operator=(System &&other)
    {
        if (other.dof_ == 0)
            other.finalize();

        links_ = std::move(other.links_);
        joints_ = std::move(other.joints_);
        kinematic_chains_ = std::move(other.kinematic_chains_);
        joints_map_ = std::move(other.joints_map_);
        links_map_ = std::move(other.links_map_);
        kinematic_chains_map_ = std::move(other.kinematic_chains_map_);
        dof_ = other.dof_;
        joint_limits_min_ = std::move(other.joint_limits_min_);
        joint_limits_max_ = std::move(other.joint_limits_max_);

        return *this;
    }

    template <class T>
    System<T>::~System() = default;

    template <class T>
    void System<T>::addJoint(std::unique_ptr<Joint<T>> &&joint)
    {
        joints_.push_back(std::move(joint));

        joints_map_.emplace(std::make_pair(joints_.back()->getName(), joints_.back().get()));
    }

    template <class T>
    void System<T>::addLink(std::unique_ptr<Link<T>> &&link)
    {
        links_.push_back(std::move(link));

        links_map_.emplace(std::make_pair(links_.back()->getName(), links_.back().get()));
    }

    template <class T>
    void System<T>::addKinematicChain(const std::string &name, std::unique_ptr<KinematicChain<T>> &&kinematic_chain)
    {
        // Remove the existing chain with that name (if any)
        auto previous = kinematic_chains_map_.find(name);
        if (previous != kinematic_chains_map_.end())
        {
            for (auto iter = kinematic_chains_.begin(); iter != kinematic_chains_.end(); ++iter)
            {
                if (iter->get() == previous->second)
                {
                    kinematic_chains_.erase(iter);
                    break;
                }
            }

            kinematic_chains_map_.erase(previous);
        }

        // Finalize the kinematic chain (if not already done)
        if (kinematic_chain->getBodies().empty())
            kinematic_chain->finalize();

        kinematic_chains_.push_back(std::move(kinematic_chain));

        kinematic_chains_map_.emplace(std::make_pair(name, kinematic_chains_.back().get()));
    }

    template <class T>
    void System<T>::finalize()
    {
        dof_ = 0;

        // Compute the axis of all links
        for (const std::unique_ptr<Joint<T>> &joint : joints_)
        {
            links_map_[joint->getChildLink()->getName()]->setAxis(
              joint->getCurrentAxis(Rotor<T>() * joint->getChildLink()->getCenterOfMass().reverse()));

            if (joint->isActuated())
            {
                dof_++;
            }
        }

        createJointLimits();

        // Create one kinematic chain for each "last joint" found in the system
        for (const std::unique_ptr<Link<T>> &link : links_)
        {
            if (link->getChildJoints().empty())
            {
                createKinematicChain(link->getParentJoint()->getName());
            }
        }
    }

    template <class T>
    std::vector<std::unique_ptr<Joint<T>>> &System<T>::getJoints()
    {
        return joints_;
    }

    template <class T>
    const std::vector<std::unique_ptr<Joint<T>>> &System<T>::getJoints() const
    {
        return joints_;
    }

    template <class T>
    const Link<T> &System<T>::getBaseLink() const
    {
        return *links_.front().get();
    }

    template <class T>
    std::vector<std::unique_ptr<Link<T>>> &System<T>::getLinks()
    {
        return links_;
    }

    template <class T>
    const std::vector<std::unique_ptr<Link<T>>> &System<T>::getLinks() const
    {
        return links_;
    }

    template <class T>
    template <class S>
    System<S> System<T>::cast()
    {
        System<S> system;

        for (unsigned j = 0; j < links_.size(); ++j)
        {
            std::unique_ptr<Link<S>> link = std::make_unique<Link<S>>();

            link->setName(links_[j]->getName());
            link->setCenterOfMass(links_[j]->getCenterOfMass());
            link->setInertia(links_[j]->getInertia());
            link->setMass(TypeTraits<S>::Value(links_[j]->getMass()));
            link->setAxis(links_[j]->getAxis());

            system.addLink(std::move(link));
        }

        for (unsigned j = 0; j < joints_.size(); ++j)
        {
            std::unique_ptr<Joint<S>> joint;

            switch (joints_[j]->getType())
            {
            case Joint<T>::Type::FIXED: {
                joint = std::make_unique<FixedJoint<S>>();

                break;
            }
            case Joint<T>::Type::REVOLUTE: {
                joint = std::make_unique<RevoluteJoint<S>>();

                static_cast<RevoluteJoint<S> *>(joint.get())->setAxis(static_cast<RevoluteJoint<T> *>(joints_[j].get())->getAxis());

                break;
            }
            case Joint<T>::Type::PRISMATIC: {
                joint = std::make_unique<PrismaticJoint<S>>();

                static_cast<PrismaticJoint<S> *>(joint.get())->setAxis(static_cast<PrismaticJoint<T> *>(joints_[j].get())->getAxis());

                break;
            }
            }

            joint->setName(joints_[j]->getName());
            joint->setFrame(joints_[j]->getFrame());
            joint->setLimits(joints_[j]->getLimits().template cast<S>());

            system.addJoint(std::move(joint));
        }

        for (const std::unique_ptr<gafro::Link<double>> &link : getLinks())
        {
            {
                auto joints = getLink(link->getName())->getChildJoints();

                for (const auto &joint : joints)
                {
                    system.getLink(link->getName())->addChildJoint(system.getJoint(joint->getName()));
                }
            }

            {
                auto parent_joint = getLink(link->getName())->getParentJoint();

                if (parent_joint)
                {
                    system.getLink(link->getName())->setParentJoint(system.getJoint(parent_joint->getName()));
                }
            }
        }

        for (const std::unique_ptr<gafro::Joint<double>> &joint : getJoints())
        {
            {
                auto link = joint->getChildLink();

                if (link)
                {
                    system.getJoint(joint->getName())->setChildLink(system.getLink(link->getName()));
                }
            }

            {
                auto link = joint->getParentLink();

                if (link)
                {
                    system.getJoint(joint->getName())->setParentLink(system.getLink(link->getName()));
                }
            }
        }

        system.finalize();

        return system;
    }

    template <class T>
    std::vector<const Joint<T> *> System<T>::getJointChain(const std::string &name) const
    {
        auto *link = getLink(name);

        std::vector<const Joint<T> *> joints;

        if (!link)
        {
            return joints;
        }

        auto joint = link->getParentJoint();

        int actuated_joints = 0;

        while (joint)
        {
            joints.push_back(joint);

            if (joint->isActuated())
            {
                actuated_joints++;
            }

            if (joint->getParentLink())
            {
                joint = joint->getParentLink()->getParentJoint();
            }
        }

        std::reverse(joints.begin(), joints.end());

        return joints;
    }

    template <class T>
    template <int dof>
    Motor<T> System<T>::computeLinkMotor(const std::string &name, const Eigen::Vector<T, dof> &position) const
    {
        std::vector<const Joint<T> *> joints = getJointChain(name);

        if (joints.empty())
        {
            return Motor<T>();
        }

        Motor<T> motor;

        int j = 0;

        for (const Joint<T> *joint : joints)
        {
            if (joint->isActuated())
            {
                motor = motor * joint->getMotor(position[j++]);
            }
            else
            {
                motor = motor * joint->getMotor(0.0);
            }
        }

        return motor;
    }

    template <class T>
    template <int dof>
    std::vector<Motor<T>> System<T>::computeJointMotors(const Eigen::Vector<T, dof> &position) const
    {
        std::vector<Motor<T>> joint_motors;

        Motor<T> motor;

        int j = 0;

        for (const auto &joint : joints_)
        {
            if (j == dof)
            {
                break;
            }

            if (joint->isActuated())
            {
                motor = motor * joint->getMotor(position[j++]);
            }
            else
            {
                motor = motor * joint->getMotor(0.0);
            }

            joint_motors.push_back(motor);
        }

        return joint_motors;
    }

    template <class T>
    template <int dof>
    Motor<T> System<T>::computeKinematicChainMotor(const std::string &name, const Eigen::Vector<T, dof> &position) const
    {
        return getKinematicChain(name)->computeMotor(position);
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, Motor, 1, dof> System<T>::computeKinematicChainAnalyticJacobian(const std::string &name,
                                                                                         const Eigen::Vector<T, dof> &position) const
    {
        return getKinematicChain(name)->computeAnalyticJacobian(position);
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> System<T>::computeKinematicChainGeometricJacobian(const std::string &name,
                                                                                                   const Eigen::Vector<T, dof> &position) const
    {
        return getKinematicChain(name)->computeGeometricJacobian(position);
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> System<T>::computeKinematicChainGeometricJacobianBody(const std::string &name,
                                                                                                       const Eigen::Vector<T, dof> &position) const
    {
        return getKinematicChain(name)->computeGeometricJacobianBody(position);
    }

    template <class T>
    Joint<T> *System<T>::getJoint(const std::string &name)
    {
        auto joint = joints_map_.find(name);

        if (joint == joints_map_.end())
        {
            return nullptr;
        }

        return joint->second;
    }

    template <class T>
    const Joint<T> *System<T>::getJoint(const std::string &name) const
    {
        auto joint = joints_map_.find(name);

        if (joint == joints_map_.end())
        {
            return nullptr;
        }

        return joint->second;
    }

    template <class T>
    const Link<T> *System<T>::getLink(const std::string &name) const
    {
        auto link = links_map_.find(name);

        if (link == links_map_.end())
        {
            return nullptr;
        }

        return link->second;
    }

    template <class T>
    Link<T> *System<T>::getLink(const std::string &name)
    {
        auto link = links_map_.find(name);

        if (link == links_map_.end())
        {
            return nullptr;
        }

        return link->second;
    }

    template <class T>
    bool System<T>::hasKinematicChain(const std::string &name) const
    {
        return kinematic_chains_map_.find(name) != kinematic_chains_map_.end();
    }

    template <class T>
    KinematicChain<T> *System<T>::getKinematicChain(const std::string &name)
    {
        auto kinematic_chain = kinematic_chains_map_.find(name);

        if (kinematic_chain == kinematic_chains_map_.end())
        {
            throw std::runtime_error("system " + name_ + " has no kinematic chain named " + name);
        }

        return kinematic_chain->second;
    }

    template <class T>
    const KinematicChain<T> *System<T>::getKinematicChain(const std::string &name) const
    {
        auto kinematic_chain = kinematic_chains_map_.find(name);

        if (kinematic_chain == kinematic_chains_map_.end())
        {
            throw std::runtime_error("system " + name_ + " has no kinematic chain named " + name);
        }

        return kinematic_chain->second;
    }

    template <class T>
    void System<T>::setName(const std::string &name)
    {
        name_ = name;
    }

    template <class T>
    const std::string &System<T>::getName() const
    {
        return name_;
    }

    template <class T>
    template <int dof>
    Eigen::Vector<T, dof> System<T>::computeInverseDynamics(const Eigen::Vector<T, dof> &position, const Eigen::Vector<T, dof> &velocity,
                                                            const Eigen::Vector<T, dof> &acceleration, const T &gravity, const Wrench<T> ee_wrench,
                                                            const std::string &kinematic_chain_name) const
    {
        static algorithm::InverseDynamics<T, dof> inverse_dynamics;

        const KinematicChain<T> *kinematic_chain = nullptr;

        if (kinematic_chain_name.empty())
        {
            for (auto &chain : kinematic_chains_)
            {
                if (chain->getDoF() == dof)
                {
                    kinematic_chain = chain.get();
                    break;
                }
            }

            if (!kinematic_chain)
            {
                throw std::runtime_error("No kinematic chain with that amount of DOF found");
            }
        }
        else
        {
            kinematic_chain = getKinematicChain(kinematic_chain_name);

            if (dof != kinematic_chain->getDoF())
            {
                throw std::runtime_error("Incorrect number of DOF");
            }
        }

        return inverse_dynamics.compute(kinematic_chain, position, velocity, acceleration, gravity);
    }

    template <class T>
    template <int dof>
    Eigen::Vector<T, dof> System<T>::computeForwardDynamics(const Eigen::Vector<T, dof> &position, const Eigen::Vector<T, dof> &velocity,
                                                            const Eigen::Vector<T, dof> &torque, const std::string &kinematic_chain_name) const
    {
        std::array<Motor<T>, dof> frames;
        std::array<Inertia<T>, dof> inertia;
        std::array<Wrench<T>, dof> bias_wrench;

        Twist<T> twist({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                         TypeTraits<T>::Zero() });
        Twist<T> twist_dt({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                            TypeTraits<T>::Value(9.81) });

        const KinematicChain<T> *kinematic_chain = nullptr;

        if (kinematic_chain_name.empty())
        {
            for (auto &chain : kinematic_chains_)
            {
                if (chain->getDoF() == dof)
                {
                    kinematic_chain = chain.get();
                    break;
                }
            }

            if (!kinematic_chain)
            {
                throw std::runtime_error("No kinematic chain with that amount of DOF found");
            }
        }
        else
        {
            kinematic_chain = getKinematicChain(kinematic_chain_name);

            if (dof != kinematic_chain->getDoF())
            {
                throw std::runtime_error("Incorrect number of DOF");
            }
        }

        const std::vector<const Joint<T> *> &joints = kinematic_chain->getActuatedJoints();
        const std::vector<std::unique_ptr<Body<T>>> &bodies = kinematic_chain->getBodies();

        for (int j = 0; j < dof; ++j)
        {
            const auto &axis = joints[j]->getCurrentAxis(gafro::Motor<T>());
            inertia[j] = bodies[j]->getInertia().transform(bodies[j]->getCenterOfMass());

            frames[j] = joints[j]->getMotor(position[j]).reverse();
            twist = frames[j].apply(twist) + Scalar<T>(velocity[j]) * axis;
            twist_dt = frames[j].apply(twist_dt) + Scalar<T>(velocity[j]) * axis.commutatorProduct(twist);

            bias_wrench[j] = inertia[j](twist_dt) - twist.commute(inertia[j](twist));
        }

        std::array<T, dof> total_torque;
        std::array<Wrench<T>, dof> axis_wrench;

        Wrench<T> wrench = Wrench<T>::Zero();
        Wrench<T> total_wrench = Wrench<T>::Zero();

        Inertia<T> articulated_body_inertia = Inertia<T>::Zero();

        Eigen::Vector<T, dof> acceleration = Eigen::Vector<T, dof>::Zero();

        for (int j = dof - 1; j > -1; --j)
        {
            const auto &axis = joints[j]->getCurrentAxis(gafro::Motor<T>());

            if (j < dof - 1)
            {
                const auto &axis = joints[j + 1]->getCurrentAxis(gafro::Motor<T>());

                articulated_body_inertia =
                  Inertia<T>({ articulated_body_inertia.getElement23() + (axis | articulated_body_inertia.getElement23()) * axis_wrench[j + 1],  //
                               articulated_body_inertia.getElement13() + (axis | articulated_body_inertia.getElement13()) * axis_wrench[j + 1],  //
                               articulated_body_inertia.getElement12() + (axis | articulated_body_inertia.getElement12()) * axis_wrench[j + 1],  //
                               articulated_body_inertia.getElement01() + (axis | articulated_body_inertia.getElement01()) * axis_wrench[j + 1],  //
                               articulated_body_inertia.getElement02() + (axis | articulated_body_inertia.getElement02()) * axis_wrench[j + 1],  //
                               articulated_body_inertia.getElement03() + (axis | articulated_body_inertia.getElement03()) * axis_wrench[j + 1] });

                articulated_body_inertia = inertia[j] + articulated_body_inertia.inverseTransform(frames[j + 1]);

                wrench = bias_wrench[j] + frames[j + 1].reverse() * wrench * frames[j + 1];
            }
            else
            {
                articulated_body_inertia = inertia[j];

                wrench = bias_wrench[j];
            }

            axis_wrench[j] = articulated_body_inertia(axis);

            T omega = TypeTraits<T>::Value(-1.0) / (axis | axis_wrench[j]).template get<blades::scalar>();

            axis_wrench[j].vector() = axis_wrench[j].vector() * omega;

            total_torque[j] = torque[j] + (axis | wrench).template get<blades::scalar>();

            if (j < dof - 1)
            {
                total_wrench =
                  frames[j + 1].reverse() *
                  (total_wrench +
                   Scalar<T>((total_wrench | joints[j + 1]->getCurrentAxis(gafro::Motor<T>())).template get<blades::scalar>() + total_torque[j + 1]) *
                     axis_wrench[j + 1]) *
                  frames[j + 1];
            }

            acceleration[j] = (total_torque[j] + (axis | total_wrench).template get<blades::scalar>()) * omega;
        }

        Twist<T> lambda;

        for (int j = 0; j < dof; ++j)
        {
            const auto &axis = joints[j]->getCurrentAxis(gafro::Motor<T>());

            if (j == 0)
            {
                lambda = Scalar<T>(acceleration[j]) * axis;
            }
            else
            {
                T uncorrected = TypeTraits<T>::copy(acceleration[j]);
                acceleration[j] += (axis_wrench[j].transform(frames[j].reverse()) | lambda).template get<blades::scalar>();

                Twist<T> transformed_lambda = lambda.transform(frames[j]);
                lambda = transformed_lambda + Scalar<T>((axis_wrench[j] | transformed_lambda).template get<blades::scalar>() + uncorrected) * axis;
            }
        }

        return acceleration;
    }

    template <class T>
    void System<T>::createKinematicChain(const std::string &joint_name)
    {
        auto kinematic_chain = std::make_unique<KinematicChain<T>>();

        std::vector<const Joint<T> *> joints;

        {
            const Joint<T> *joint = getJoint(joint_name);

            while (joint)
            {
                joints.push_back(joint);

                if (joint->getParentLink())
                {
                    joint = joint->getParentLink()->getParentJoint();
                }
            }

            std::reverse(joints.begin(), joints.end());

            for (const Joint<T> *joint : joints)
            {
                if (joint->getType() == Joint<T>::Type::FIXED)
                {
                    kinematic_chain->addFixedMotor(joint->getMotor(TypeTraits<T>::Zero()));
                }
                else
                {
                    kinematic_chain->addActuatedJoint(joint);
                }
            }
        }

        kinematic_chain->finalize();

        this->addKinematicChain(joint_name, std::move(kinematic_chain));
    }

    template <class T>
    void System<T>::createJointLimits()
    {
        Vector joint_limits_min = Vector::Zero(dof_);
        Vector joint_limits_max = Vector::Zero(dof_);

        int i = 0;
        for (const std::unique_ptr<Joint<T>> &joint : joints_)
        {
            if (joint->isActuated())
            {
                joint_limits_min[i] = joint->getLimits().position_lower;
                joint_limits_max[i] = joint->getLimits().position_upper;
                ++i;
            }
        }

        setJointLimits(joint_limits_min, joint_limits_max);
    }

    template <class T>
    typename System<T>::Vector System<T>::getRandomConfiguration() const
    {
        Vector random = Vector::Random(dof_);

        return joint_limits_min_ + (random.array().abs() * (joint_limits_max_ - joint_limits_min_).array()).matrix();
    }

    template <class T>
    void System<T>::setJointLimits(const Vector &joint_limits_min, const Vector &joint_limits_max)
    {
        joint_limits_min_ = joint_limits_min;
        joint_limits_max_ = joint_limits_max;
    }

    template <class T>
    const typename System<T>::Vector &System<T>::getJointLimitsMin() const
    {
        return joint_limits_min_;
    }

    template <class T>
    const typename System<T>::Vector &System<T>::getJointLimitsMax() const
    {
        return joint_limits_max_;
    }

    template <class T>
    bool System<T>::isJointPositionFeasible(const Vector &position) const
    {
        return (position.array() > joint_limits_min_.array()).all()  //
               && (position.array() < joint_limits_max_.array()).all();
    }

}  // namespace gafro