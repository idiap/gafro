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

#include <gafro/algebra/MultivectorMatrix.hpp>
//
#include <gafro/robot/Hand.hpp>

namespace gafro
{

    template <class T, int n_fingers, int dof>
    Hand<T, n_fingers, dof>::Hand(Hand &&hand)
    {
        *this = std::move(hand);
    }

    template <class T, int n_fingers, int dof>
    Hand<T, n_fingers, dof> &Hand<T, n_fingers, dof>::operator=(Hand &&hand)
    {
        this->System<T>::operator=(std::move(hand));
        finger_tip_names_ = std::move(hand.finger_tip_names_);
    }

    template <class T, int n_fingers, int dof>
    Hand<T, n_fingers, dof>::Hand(System<T> &&system, const std::array<std::string, n_fingers> &finger_tip_names)
      : System<T>(std::move(system)), finger_tip_names_(finger_tip_names)
    {
        for (int j = 0; j < n_fingers; ++j)
        {
            this->createKinematicChain(finger_tip_names[j]);
        }
    }

    template <class T, int n_fingers, int dof>
    Hand<T, n_fingers, dof>::~Hand() = default;

    //

    template <class T, int n_fingers, int dof>
    System<T> &Hand<T, n_fingers, dof>::getSystem()
    {
        return *this;
    }

    template <class T, int n_fingers, int dof>
    const System<T> &Hand<T, n_fingers, dof>::getSystem() const
    {
        return *this;
    }

    //

    template <class T, int n_fingers, int dof>
    Motor<T> Hand<T, n_fingers, dof>::getFingerMotor(const unsigned &id, const Eigen::Vector<T, dof> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainMotor(finger_tip_names_[id], position);
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, Motor, 1, n_fingers> Hand<T, n_fingers, dof>::getFingerMotors(const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers> motors;

        for (int j = 0; j < n_fingers; ++j)
        {
            motors.setCoefficient(0, j, getFingerMotor(j, position.middleRows(j * dof, dof)));
        }

        return motors;
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, Motor, 1, dof> Hand<T, n_fingers, dof>::getFingerAnalyticJacobian(const unsigned &id,
                                                                                           const Eigen::Vector<T, dof> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainAnalyticJacobian(finger_tip_names_[id], position);
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Hand<T, n_fingers, dof>::getFingerGeometricJacobian(const unsigned &id,
                                                                                                     const Eigen::Vector<T, dof> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainGeometricJacobian(finger_tip_names_[id], position);
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Hand<T, n_fingers, dof>::getFingerGeometricJacobian(const unsigned &id,
                                                                                                     const Eigen::Vector<T, dof> &position,
                                                                                                     const Motor<T> &motor) const
    {
        return getFingerGeometricJacobian(id, position).transform(motor.reverse());
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, Motor, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getAnalyticJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers * dof> jacobian;

        for (int j = 0; j < n_fingers; ++j)
        {
            MultivectorMatrix<T, Motor, 1, dof> finger_jacobian = getFingerAnalyticJacobian(j, position.middleRows(j * dof, dof));

            for (int k = 0; k < dof; ++k)
            {
                jacobian.setCoefficient(0, j * dof + k, finger_jacobian.getCoefficient(0, k));
            }
        }

        return jacobian;
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getGeometricJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> jacobian;

        for (int j = 0; j < n_fingers; ++j)
        {
            MultivectorMatrix<T, MotorGenerator, 1, dof> finger_jacobian = getFingerGeometricJacobian(j, position.middleRows(j * dof, dof));

            for (int k = 0; k < dof; ++k)
            {
                jacobian.setCoefficient(0, j * dof + k, finger_jacobian.getCoefficient(0, k));
            }
        }

        return jacobian;
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getGeometricJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position, const Motor<T> &motor) const
    {
        return getGeometricJacobian(position).transform(motor.reverse());
    }

    template <class T, int n_fingers, int dof>
    Motor<T> Hand<T, n_fingers, dof>::getMeanMotor(const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        typename Motor<T>::Generator mean_bivector;

        for (const auto &m : getFingerMotors(position).asVector())
        {
            mean_bivector += m.log().evaluate();
        }

        return Motor<T>::exp(mean_bivector / static_cast<T>(n_fingers));
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, Motor, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getMeanMotorAnalyticJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        Motor<T> mean_motor = getMeanMotor(position);

        Eigen::Matrix<T, 8, 6> exp_jacobian = Motor<T>::Exponential::getJacobian(mean_motor.log());

        MultivectorMatrix<T, Motor, 1, n_fingers * dof> jacobian;

        for (int j = 0; j < n_fingers; ++j)
        {
            MultivectorMatrix<T, Motor, 1, dof> finger_jacobian = getFingerAnalyticJacobian(j, position.middleRows(j * dof, dof));

            Eigen::Matrix<T, 6, 8> log_jacobian = Motor<T>::Logarithm::getJacobian(getFingerMotor(j, position.middleRows(j * dof, dof)));

            for (int k = 0; k < dof; ++k)
            {
                typename Motor<T>::Parameters parameters =
                  exp_jacobian * log_jacobian / static_cast<double>(n_fingers) * finger_jacobian.getCoefficient(0, k).vector();

                jacobian.setCoefficient(0, j * dof + k, Motor<T>(parameters));
            }
        }

        return jacobian;
    }

    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getMeanMotorGeometricJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> jacobian;

        MultivectorMatrix<T, Motor, 1, n_fingers * dof> analytic_jacobian = getMeanMotorAnalyticJacobian(position);

        for (int j = 0; j < n_fingers * dof; ++j)
        {
            jacobian.setCoefficient(0, j, -2.0 * analytic_jacobian.getCoefficient(0, j) * getMeanMotor(position).reverse());
        }

        return jacobian;
    }

    //

}  // namespace gafro