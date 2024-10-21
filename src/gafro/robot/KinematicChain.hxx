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

#include <gafro/robot/KinematicChain.hpp>

namespace gafro
{

    template <class T>
    KinematicChain<T>::KinematicChain(const std::string &name) : name_(name)
    {}

    template <class T>
    KinematicChain<T>::KinematicChain(KinematicChain &&other)
    {
        *this = std::move(other);
    }

    template <class T>
    KinematicChain<T>::~KinematicChain() = default;

    template <class T>
    KinematicChain<T> &KinematicChain<T>::operator=(KinematicChain &&other)
    {
        actuated_joints_ = std::move(other.actuated_joints_);
        fixed_motors_ = std::move(other.fixed_motors_);
        bodies_ = std::move(other.bodies_);
        name_ = std::move(other.name_);

        return *this;
    }

    template <class T>
    int KinematicChain<T>::getDoF() const
    {
        return actuated_joints_.size();
    }

    template <class T>
    const std::string &KinematicChain<T>::getName() const
    {
        return name_;
    }

    template <class T>
    void KinematicChain<T>::addActuatedJoint(const Joint<T> *joint)
    {
        actuated_joints_.push_back(joint);
    }

    template <class T>
    void KinematicChain<T>::addFixedMotor(const Motor<T> &motor)
    {
        int index = static_cast<int>(actuated_joints_.size()) - 1;

        if (fixed_motors_.find(index) == fixed_motors_.end())
        {
            fixed_motors_.emplace(std::make_pair(index, motor));
        }
        else
        {
            fixed_motors_[index] *= motor;
        }
    }

    template <class T>
    void KinematicChain<T>::setFixedMotors(const std::map<int, Motor<T>> &fixed_motors)
    {
        fixed_motors_ = fixed_motors;
    }

    template <class T>
    const std::map<int, Motor<T>> &KinematicChain<T>::getFixedMotors() const
    {
        return fixed_motors_;
    }

    template <class T>
    const std::vector<const Joint<T> *> &KinematicChain<T>::getActuatedJoints() const
    {
        return actuated_joints_;
    }

    template <class T>
    const std::vector<std::unique_ptr<Body<T>>> &KinematicChain<T>::getBodies() const
    {
        return bodies_;
    }

    template <class T>
    template <int dof>
    Motor<T> KinematicChain<T>::computeMotor(const Eigen::Vector<T, dof> &position) const
    {
        if (dof != actuated_joints_.size())
        {
            throw std::runtime_error("kinematic chain has not enough dof!");
        }

        Motor<T> motor;

        for (int i = 0; i < dof; ++i)
        {
            motor = motor * computeMotor(i, position[i]);
        }

        return motor;
    }

    template <class T>
    Motor<T> KinematicChain<T>::computeMotor(const int &index, const T &position) const
    {
        Motor<T> motor = actuated_joints_[index]->getMotor(position);

        if ((index == 0) && (fixed_motors_.find(-1) != fixed_motors_.end()))
        {
            motor = fixed_motors_.at(-1) * motor;
        }

        if (fixed_motors_.find(index) != fixed_motors_.end())
        {
            motor = motor * fixed_motors_.at(index);
        }

        return motor;
    }

    template <class T>
    Motor<T> KinematicChain<T>::computeMotorDerivative(const int &index, const T &position) const
    {
        Motor<T> motor = actuated_joints_[index]->getMotorDerivative(position);

        if ((index == 0) && (fixed_motors_.find(-1) != fixed_motors_.end()))
        {
            motor = fixed_motors_.at(-1) * motor;
        }

        if (fixed_motors_.find(index) != fixed_motors_.end())
        {
            motor = motor * fixed_motors_.at(index);
        }

        return motor;
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, Motor, 1, dof> KinematicChain<T>::computeAnalyticJacobian(const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, dof> jacobian;

        for (unsigned i = 0; i < dof; ++i)
        {
            Motor<T> motor = computeMotor(i, position[i]);

            for (unsigned j = 0; j < dof; ++j)
            {
                if (j == i)
                {
                    jacobian.getCoefficient(0, j) *= computeMotorDerivative(i, position[i]);
                }
                else
                {
                    jacobian.getCoefficient(0, j) *= motor;
                }
            }
        }

        return jacobian;
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> KinematicChain<T>::computeGeometricJacobian(const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian;

        Motor<T> joint_motor;

        if (fixed_motors_.find(-1) != fixed_motors_.end())
        {
            joint_motor = fixed_motors_.at(-1);
        }

        for (unsigned i = 0; i < dof; ++i)
        {
            Motor<T> motor = joint_motor * actuated_joints_[i]->getFrame();

            jacobian.getCoefficient(0, i) = actuated_joints_[i]->getCurrentAxis(motor);

            if (i == 0)
                joint_motor = computeMotor(i, position[i]);
            else
                joint_motor *= computeMotor(i, position[i]);
        }

        return jacobian;
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> KinematicChain<T>::computeGeometricJacobianBody(const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian;

        auto fixed_motor = fixed_motors_.find(dof - 1);
        Motor<T> joint_motor = (fixed_motor != fixed_motors_.end() ? fixed_motor->second : Motor<T>());

        for (int i = dof - 1; i > -1; --i)
        {
            jacobian.getCoefficient(0, i) = actuated_joints_[i]->getCurrentAxis(joint_motor.reverse());

            joint_motor = actuated_joints_[i]->getMotor(position.coeff(i, 0)) * joint_motor;
        }

        return jacobian;
    }

    template <class T>
    template <int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> KinematicChain<T>::computeKinematicChainGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof> &position, const Eigen::Vector<T, dof> &velocity, const Motor<T> &reference) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian = computeGeometricJacobian(position);
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian_time_derivative;

        MotorGenerator<T> twist;

        for (int i = 0; i < dof; ++i)
        {
            twist = twist + Scalar<T>(velocity[i]) * jacobian.getCoefficient(0, i);

            MotorGenerator<T> b1 = jacobian.getCoefficient(0, i);
            MotorGenerator<T> b2 = twist;

            jacobian_time_derivative.setCoefficient(0, i, b1.commute(b2));
        }

        return jacobian_time_derivative.transform(reference.reverse());
    }

    template <class T>
    template <int dof>
    Eigen::Matrix<T, dof, dof> KinematicChain<T>::computeMassMatrix(const Eigen::Vector<T, dof> &position) const
    {
        Eigen::Matrix<T, 7, 7> mass_matrix = Eigen::Matrix<T, 7, 7>::Zero();

        auto gj = this->computeGeometricJacobian(position);

        Motor<T> m;

        for (int j = 0; j < 7; ++j)
        {
            m *= actuated_joints_[j]->getMotor(position[j]);

            Inertia<T> inertia = bodies_[j]->getInertia().transform(m * bodies_[j]->getCenterOfMass());

            for (int k = 0; k < j + 1; ++k)
            {
                for (int l = 0; l < j + 1; ++l)
                {
                    mass_matrix.coeffRef(k, l) += -(inertia(gj.getCoefficient(0, l)) | gj.getCoefficient(0, k)).template get<blades::scalar>();
                }
            }
        }

        return mass_matrix;
    }

    template <class T>
    void KinematicChain<T>::finalize()
    {
        bodies_.clear();

        for (size_t i = 0; i < actuated_joints_.size(); ++i)
        {
            createBody(actuated_joints_[i]);
        }
    }

    template <class T>
    void KinematicChain<T>::createBody(const Joint<T> *startJoint)
    {
        typename Motor<T>::Generator com_generator = Motor<T>::Generator::Zero();
        Inertia<T> inertia = Inertia<T>::Zero();
        T total_mass = TypeTraits<T>::Zero();

        std::function<void(const Link<T> *link, const Motor<T> &motor)> addLinkInertia;

        addLinkInertia = [&inertia, &com_generator, &addLinkInertia, &total_mass, this](const Link<T> *link, const Motor<T> &motor) {
            if (TypeTraits<T>::greater(link->getMass(), TypeTraits<T>::Zero()))
            {
                Motor<T> com_link = motor * link->getCenterOfMass();

                inertia += link->getInertia().transform(com_link);

                com_generator = com_generator + Scalar<T>(link->getMass()) * com_link.log().evaluate().getTranslatorGenerator();

                total_mass += link->getMass();
            }

            for (const auto *child_joint : link->getChildJoints())
            {
                if (child_joint->isActuated() &&
                    (std::find(this->actuated_joints_.begin(), this->actuated_joints_.end(), child_joint) != this->actuated_joints_.end()))
                {
                    continue;
                }

                auto child_link = child_joint->getChildLink();
                if (child_link)
                {
                    addLinkInertia(child_link, motor * child_joint->getMotor(TypeTraits<T>::Zero()));
                }
            }
        };

        addLinkInertia(startJoint->getChildLink(), Motor<T>());

        com_generator /= total_mass;

        Motor<T> com = Motor<T>::exp(com_generator);
        inertia = inertia.inverseTransform(com);

        auto body = std::make_unique<Body<T>>();
        body->setCenterOfMass(com.getTranslator());
        body->setInertia(inertia);
        body->setAxis(startJoint->getCurrentAxis(Rotor<T>() * body->getCenterOfMass().reverse()));
        body->setMass(total_mass);

        bodies_.push_back(std::move(body));
    }

}  // namespace gafro