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

        for (const std::unique_ptr<gafro::Joint<double>> &joint : system_joints)
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
    KinematicChain<double> *Manipulator<T, dof>::getEEKinematicChain()
    {
        return getSystem().getKinematicChain(ee_joint_name_);
    }

    template <class T, int dof>
    const KinematicChain<double> *Manipulator<T, dof>::getEEKinematicChain() const
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
    MultivectorMatrix<T, MotorGenerator, 1, dof> Manipulator<T, dof>::getEEFrameJacobian(const Vector &position) const
    {
        return getSystem().computeKinematicChainGeometricJacobianBody(ee_joint_name_, position);
    }

    template <class T, int dof>
    template <class Primitive>
    typename SandwichProduct<Primitive, Motor<T>>::Type::template Matrix<1, dof> Manipulator<T, dof>::getEEPrimitiveJacobian(
      const Vector &position, const Primitive &primitive) const
    {
        using Result = typename SandwichProduct<Primitive, Motor<T>>::Type;

        Motor<double> ee_motor = getEEMotor(position);
        auto ee_jacobian = getEEAnalyticJacobian(position);

        auto jacobian = Result::template CreateMatrix<1, dof>();

        for (unsigned i = 0; i < dof; ++i)
        {
            jacobian.setCoefficient(0, i,
                                    Primitive(ee_jacobian.getCoefficient(0, i) * primitive * ee_motor.reverse() +  //
                                              ee_motor * primitive * ee_jacobian.getCoefficient(0, i).reverse()));
        }

        return jacobian;
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

}  // namespace gafro