// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>
#include <array>
#include <functional>
#include <memory>
#include <vector>
//
#include <gafro/gafro_package_config.hpp>
//
#include <gafro/robot/Joint.hpp>
#include <gafro/robot/Link.hpp>
#include <gafro/robot/RevoluteJoint.hpp>
#include <gafro/robot/System.hpp>

namespace gafro
{

    template <class T>
    class Twist;

    template <class T>
    class Wrench;

    template <class T, int dof>
    class Manipulator : private System<T>
    {
      public:
        using Vector = Eigen::Matrix<T, dof, 1>;

        Manipulator() = delete;

        Manipulator(const Manipulator &other) = delete;

        System<T> &operator=(System<T> &&manipulator) = delete;

        Manipulator(Manipulator &&manipulator);

        Manipulator &operator=(Manipulator &&manipulator);

        Manipulator(System<T> &&system, const std::string &ee_joint_name = "endeffector");

        virtual ~Manipulator();

        //

        System<T> &getSystem();

        const System<T> &getSystem() const;

        //

        using System<T>::getJoint;
        using System<T>::getLink;
        using System<T>::computeLinkMotor;

        //

        Vector getRandomConfiguration() const;

        Vector getJointLimitsMin() const;

        Vector getJointLimitsMax() const;

        //

        KinematicChain<T> *getEEKinematicChain();

        const KinematicChain<T> *getEEKinematicChain() const;

        //

        Motor<T> getEEMotor(const Vector &position) const;

        MultivectorMatrix<T, Motor, 1, dof> getEEAnalyticJacobian(const Vector &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getEEGeometricJacobian(const Vector &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getGeometricJacobian(const Vector &position, const Motor<T> &reference) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getGeometricJacobianTimeDerivative(const Vector &position, const Vector &velocity,
                                                                                        const Motor<T> &reference) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getEEFrameJacobian(const Vector &position) const;

        template <class Primitive>
        typename Primitive::Type::template Matrix<1, dof> getEEPrimitiveJacobian(const Vector &position, const Primitive &primitive) const;

        //

        Eigen::Matrix<T, 6, 6> getEEVelocityManipulability(const Vector &position) const;

        Eigen::Matrix<T, 6, 6> getEEForceManipulability(const Vector &position) const;

        Eigen::Matrix<T, 6, 6> getEEDynamicManipulability(const Vector &position) const;

        Eigen::Matrix<T, 7, 7> getEEKinematicNullspaceProjector(const Vector &position) const;

        //

        Vector getJointTorques(const Vector &position,                         //
                               const Vector &velocity,                         //
                               const Vector &acceleration,                     //
                               const T &gravity = TypeTraits<T>::Value(9.81),  //
                               const Wrench<T> ee_wrench = Wrench<T>::Zero()) const;

        Vector getJointAccelerations(const Vector &position,  //
                                     const Vector &velocity,  //
                                     const Vector &torque) const;

        Eigen::Matrix<T, dof, dof> getMassMatrix(const Eigen::Vector<T, dof> &position) const;

      private:
        std::string ee_joint_name_;

      public:
        using Ptr = std::shared_ptr<Manipulator>;
        using ConstPtr = std::shared_ptr<const Manipulator>;
    };
}  // namespace gafro