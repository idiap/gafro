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

        //

        KinematicChain<double> *getEEKinematicChain();

        const KinematicChain<double> *getEEKinematicChain() const;

        //

        Motor<T> getEEMotor(const Vector &position) const;

        MultivectorMatrix<T, Motor, 1, dof> getEEAnalyticJacobian(const Vector &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getEEGeometricJacobian(const Vector &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getGeometricJacobian(const Vector &position, const Motor<T> &reference) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getEEFrameJacobian(const Vector &position) const;

        template <class Primitive>
        typename SandwichProduct<Primitive, Motor<T>>::Type::template Matrix<1, dof> getEEPrimitiveJacobian(const Vector &position,
                                                                                                            const Primitive &primitive) const;

        //

        Vector getJointTorques(const Vector &position,                         //
                               const Vector &velocity,                         //
                               const Vector &acceleration,                     //
                               const T &gravity = TypeTraits<T>::Value(9.81),  //
                               const Wrench<T> ee_wrench = Wrench<T>::Zero()) const;

        Vector getJointAccelerations(const Vector &position,  //
                                     const Vector &velocity,  //
                                     const Vector &torque) const;

      private:
        std::string ee_joint_name_;

      public:
        using Ptr = std::shared_ptr<Manipulator>;
        using ConstPtr = std::shared_ptr<const Manipulator>;
    };
}  // namespace gafro