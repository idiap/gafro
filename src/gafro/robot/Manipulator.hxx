// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/gafro_package_config.hpp>
//
#include <gafro/algebra/MultivectorMatrix.hpp>
//
#include <gafro/physics/Twist.hxx>
#include <gafro/physics/Wrench.hxx>
//
#include <algorithm>
#include <gafro/robot/Manipulator.hpp>

namespace gafro
{

    template <class T, int dof>
    Manipulator<T, dof>::Manipulator(System<T> &&system, const std::string &ee_joint_name)
      : System<T>(std::move(system)), ee_joint_name_(ee_joint_name)
    {
        this->createKinematicChain(ee_joint_name);
    }

    template <class T, int dof>
    Manipulator<T, dof>::Manipulator(Manipulator &&manipulator)
    {
        *this = std::move(manipulator);
    }

    template <class T, int dof>
    Manipulator<T, dof> &Manipulator<T, dof>::operator=(Manipulator &&manipulator)
    {
        this->System<T>::operator=(std::move(manipulator));
        ee_joint_name_ = std::move(manipulator.ee_joint_name_);

        return *this;
    }

    template <class T, int dof>
    Manipulator<T, dof>::~Manipulator()
    {}

    template <class T, int dof>
    System<T> &Manipulator<T, dof>::getSystem()
    {
        return *this;
    }

    template <class T, int dof>
    const System<T> &Manipulator<T, dof>::getSystem() const
    {
        return *this;
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::Vector Manipulator<T, dof>::getRandomConfiguration() const
    {
        // We need to filter the result of System::getRandomConfiguration() to keep only the joints
        // we care about, in the correct order

        Vector configuration;

        auto system_configuration = System<T>::getRandomConfiguration();
        auto kinematic_chain = getEEKinematicChain();
        auto joints = kinematic_chain->getActuatedJoints();
        auto &system_joints = System<T>::getJoints();

        std::vector<std::string> names;

        for (const std::unique_ptr<gafro::Joint<T>> &joint : system_joints)
        {
            if (joint->isActuated())
                names.push_back(joint->getName());
        }

        for (size_t i = 0; i < dof; ++i)
        {
            auto joint = joints[i];

            auto it = find(names.begin(), names.end(), joint->getName());
            size_t j = it - names.begin();

            configuration[i] = system_configuration[j];
        }

        return configuration;
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::Vector Manipulator<T, dof>::getJointLimitsMin() const
    {
        Vector joint_limits;

        typename System<T>::Vector joint_limits_system = System<T>::getJointLimitsMin();

        for (unsigned i = 0; i < dof; ++i)
        {
            joint_limits[i] = joint_limits_system[getEEKinematicChain()->getActuatedJoints()[i]->getIndex()];
        }

        return joint_limits;
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::Vector Manipulator<T, dof>::getJointLimitsMax() const
    {
        Vector joint_limits;

        typename System<T>::Vector joint_limits_system = System<T>::getJointLimitsMax();

        for (unsigned i = 0; i < dof; ++i)
        {
            joint_limits[i] = joint_limits_system[getEEKinematicChain()->getActuatedJoints()[i]->getIndex()];
        }

        return joint_limits;
    }

    template <class T, int dof>
    KinematicChain<T> *Manipulator<T, dof>::getEEKinematicChain()
    {
        return getSystem().getKinematicChain(ee_joint_name_);
    }

    template <class T, int dof>
    const KinematicChain<T> *Manipulator<T, dof>::getEEKinematicChain() const
    {
        return getSystem().getKinematicChain(ee_joint_name_);
    }

    template <class T, int dof>
    Motor<T> Manipulator<T, dof>::getEEMotor(const Vector &position) const
    {
        return getSystem().computeKinematicChainMotor(ee_joint_name_, position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> Manipulator<T, dof>::getEEAnalyticJacobian(const Vector &position) const
    {
        return getSystem().computeKinematicChainAnalyticJacobian(ee_joint_name_, position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Manipulator<T, dof>::getEEGeometricJacobian(const Vector &position) const
    {
        return getSystem().computeKinematicChainGeometricJacobian(ee_joint_name_, position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Manipulator<T, dof>::getGeometricJacobian(const Vector &position, const Motor<T> &reference) const
    {
        auto jacobian = getSystem().computeKinematicChainGeometricJacobian(ee_joint_name_, position);

        for (unsigned i = 0; i < dof; ++i)
        {
            jacobian.setCoefficient(0, i, reference.reverse() * jacobian.getCoefficient(0, i) * reference);
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Manipulator<T, dof>::getGeometricJacobianTimeDerivative(const Vector &position,
                                                                                                         const Vector &velocity,
                                                                                                         const Motor<T> &reference) const
    {
        return getSystem().computeKinematicChainGeometricJacobianTimeDerivative(ee_joint_name_, position, velocity, reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Manipulator<T, dof>::getEEFrameJacobian(const Vector &position) const
    {
        return getSystem().computeKinematicChainGeometricJacobianBody(ee_joint_name_, position);
    }

    template <class T, int dof>
    template <class Primitive>
    typename Primitive::Type::template Matrix<1, dof> Manipulator<T, dof>::getEEPrimitiveJacobian(const Vector &position,
                                                                                                  const Primitive &primitive) const
    {
        Motor<T> ee_motor = getEEMotor(position);
        auto ee_jacobian = getEEAnalyticJacobian(position);

        typename Primitive::Type::template Matrix<1, dof> jacobian;

        for (unsigned i = 0; i < dof; ++i)
        {
            jacobian.setCoefficient(0, i,
                                    Primitive(ee_jacobian.getCoefficient(0, i) * primitive * ee_motor.reverse() +  //
                                              ee_motor * primitive * ee_jacobian.getCoefficient(0, i).reverse()));
        }

        return jacobian;
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> Manipulator<T, dof>::getEEVelocityManipulability(const Vector &position) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getEEGeometricJacobian(position).embed();

        return jacobian * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> Manipulator<T, dof>::getEEForceManipulability(const Vector &position) const
    {
        return getEEVelocityManipulability(position).inverse();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> Manipulator<T, dof>::getEEDynamicManipulability(const Vector &position) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getEEGeometricJacobian(position).embed();
        Eigen::Matrix<T, dof, dof> mass_matrix = getMassMatrix(position);

        return jacobian * (mass_matrix.transpose() * mass_matrix).inverse() * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, dof, dof> Manipulator<T, dof>::getEEKinematicNullspaceProjector(const Vector &position) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getEEGeometricJacobian(position).embed();
        Eigen::Matrix<T, dof, 6> inverse_jacobian =
          (jacobian.transpose() * jacobian + 1e-5 * Eigen::Matrix<T, dof, dof>::Identity()).inverse() * jacobian.transpose();

        return Eigen::Matrix<T, dof, dof>::Identity() - inverse_jacobian * jacobian;
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::Vector Manipulator<T, dof>::getJointTorques(const Vector &position,      //
                                                                              const Vector &velocity,      //
                                                                              const Vector &acceleration,  //
                                                                              const T &gravity,            //
                                                                              const Wrench<T> ee_wrench) const
    {
        return getSystem().computeInverseDynamics(position, velocity, acceleration, gravity, ee_wrench, ee_joint_name_);
    }

    template <class T, int dof>
    typename Manipulator<T, dof>::Vector Manipulator<T, dof>::getJointAccelerations(const Vector &position,  //
                                                                                    const Vector &velocity,  //
                                                                                    const Vector &torque) const
    {
        return getSystem().computeForwardDynamics(position, velocity, torque, ee_joint_name_);
    }

    template <class T, int dof>
    Eigen::Matrix<T, dof, dof> Manipulator<T, dof>::getMassMatrix(const Eigen::Vector<T, dof> &position) const
    {
        return getSystem().computeKinematicChainMassMatrix(ee_joint_name_, position);
    }

}  // namespace gafro