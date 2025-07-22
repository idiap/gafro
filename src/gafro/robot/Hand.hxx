// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/MultivectorMatrix.hpp>
//
#include <gafro/robot/Hand.hpp>

namespace gafro
{

    template <class T, int... fingers>
    Hand<T, fingers...>::Hand(Hand &&hand)
    {
        *this = std::move(hand);
    }

    template <class T, int... fingers>
    Hand<T, fingers...> &Hand<T, fingers...>::operator=(Hand &&hand)
    {
        this->System<T>::operator=(std::move(hand));
        finger_tip_names_ = std::move(hand.finger_tip_names_);
    }

    template <class T, int... fingers>
    Hand<T, fingers...>::Hand(System<T> &&system, const std::array<std::string, n_fingers> &finger_tip_names)
      : System<T>(std::move(system)), finger_tip_names_(finger_tip_names)
    {
        for (int j = 0; j < n_fingers; ++j)
        {
            this->createKinematicChain(finger_tip_names[j]);
        }
    }

    template <class T, int... fingers>
    Hand<T, fingers...>::~Hand() = default;

    //

    template <class T, int... fingers>
    System<T> &Hand<T, fingers...>::getSystem()
    {
        return *this;
    }

    template <class T, int... fingers>
    const System<T> &Hand<T, fingers...>::getSystem() const
    {
        return *this;
    }

    //

    template <class T, int... fingers>
    template <int id>
    Motor<T> Hand<T, fingers...>::getFingerMotor(const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainMotor(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerAnalyticJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainAnalyticJacobian(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerGeometricJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainGeometricJacobian(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerGeometricJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position, const Motor<T> &motor) const
    {
        return getFingerGeometricJacobian<id>(position).transform(motor.reverse());
    }

    //

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::n_fingers> Hand<T, fingers...>::getFingerMotors(const Eigen::Vector<T, dof> &position) const
    {
        FingerMotorFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.motors;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Point, 1, Hand<T, fingers...>::n_fingers> Hand<T, fingers...>::getFingerPoints(const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers> motors = getFingerMotors(position);

        MultivectorMatrix<T, Point, 1, n_fingers> points;

        for (int j = 0; j < n_fingers; ++j)
        {
            points.setCoefficient(0, j, motors.getCoefficient(0, j).apply(Point<T>()));
        }

        return points;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getAnalyticJacobian(const Eigen::Vector<T, dof> &position) const
    {
        FingerAnalyticJacobianFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getGeometricJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        FingerGeometricJacobianFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getGeometricJacobian(const Eigen::Vector<T, dof> &position,
                                                                                                                const Motor<T> &motor) const
    {
        return getGeometricJacobian(position).transform(motor.reverse());
    }

    template <class T, int... fingers>
    Motor<T> Hand<T, fingers...>::getMeanMotor(const Eigen::Vector<T, dof> &position) const
    {
        typename Motor<T>::Generator mean_bivector;

        for (const auto &m : getFingerMotors(position).asVector())
        {
            mean_bivector += m.log().evaluate();
        }

        return Motor<T>::exp(mean_bivector / static_cast<T>(n_fingers));
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getMeanMotorAnalyticJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers> motors = getFingerMotors(position);

        Motor<T> mean_motor = getMeanMotor(position);

        Eigen::Matrix<T, 8, 6> exp_jacobian = Motor<T>::Exponential::getJacobian(mean_motor.log());

        MultivectorMatrix<T, Motor, 1, dof> jacobian;

        MultivectorMatrix<T, Motor, 1, dof> finger_jacobian = getAnalyticJacobian(position);

        for (int j = 0; j < n_fingers; ++j)
        {
            Eigen::Matrix<T, 6, 8> log_jacobian = Motor<T>::Logarithm::getJacobian(motors.getCoefficient(0, j));
            for (int k = 0; k < finger_dof[j]; ++k)
            {
                typename Motor<T>::Parameters parameters =
                  exp_jacobian * log_jacobian / static_cast<double>(n_fingers) * finger_jacobian.getCoefficient(0, k).vector();

                jacobian.setCoefficient(0, std::accumulate(finger_dof.begin(), finger_dof.begin() + j, 0) + k, Motor<T>(parameters));
            }
        }

        return jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getMeanMotorGeometricJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian;

        Motor<T> mean_motor = getMeanMotor(position);

        MultivectorMatrix<T, Motor, 1, dof> analytic_jacobian = getMeanMotorAnalyticJacobian(position);

        for (int j = 0; j < dof; ++j)
        {
            jacobian.setCoefficient(0, j, -2.0 * analytic_jacobian.getCoefficient(0, j) * mean_motor.reverse());
        }

        return jacobian;
    }

    //

}  // namespace gafro