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
    MultivectorMatrix<T, Point, 1, n_fingers> Hand<T, n_fingers, dof>::getFingerPoints(const Eigen::Vector<T, n_fingers * dof> &position) const
    {
        MultivectorMatrix<T, Point, 1, n_fingers> points;

        for (int j = 0; j < n_fingers; ++j)
        {
            points.setCoefficient(0, j, getFingerMotor(j, position.middleRows(j * dof, dof)).apply(Point<T>()));
        }

        return points;
    }

    template <class T, int n_fingers, int dof>
    Sphere<T> Hand<T, n_fingers, dof>::getFingerSphere(const Eigen::Vector<T, n_fingers * dof> &position) const
        requires(n_fingers == 4)
    {
        MultivectorMatrix<T, Point, 1, n_fingers> points = getFingerPoints(position);

        return Sphere<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2), points.getCoefficient(0, 3));
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
    template <class T, int n_fingers, int dof>
    MultivectorMatrix<T, Sphere, 1, n_fingers * dof> Hand<T, n_fingers, dof>::getFingerSphereJacobian(
      const Eigen::Vector<T, n_fingers * dof> &position) const
        requires(n_fingers == 4)
    {
        auto finger_motors_1 = getFingerMotors(position).asVector();
        auto finger_points_1 = getFingerPoints(position).asVector();
        auto finger_0_jacobian_1 = getFingerAnalyticJacobian(0, position.middleRows(0, 4));
        auto finger_1_jacobian_1 = getFingerAnalyticJacobian(1, position.middleRows(4, 4));
        auto finger_2_jacobian_1 = getFingerAnalyticJacobian(2, position.middleRows(8, 4));
        auto finger_3_jacobian_1 = getFingerAnalyticJacobian(3, position.middleRows(12, 4));

        auto j_0_1 = finger_0_jacobian_1 * (Point<T>() * finger_motors_1[0].reverse()).evaluate();
        auto j_0_2 = (finger_motors_1[0] * Point<T>()).evaluate() * finger_0_jacobian_1.reverse();
        auto finger_0_sphere_jacobian = ((j_0_1 + j_0_2) ^ (finger_points_1[1] ^ finger_points_1[2] ^ finger_points_1[3]).evaluate()).normalized();

        auto j_1_1 = finger_1_jacobian_1 * (Point<T>() * finger_motors_1[1].reverse()).evaluate();
        auto j_1_2 = (finger_motors_1[1] * Point<T>()).evaluate() * finger_1_jacobian_1.reverse();
        auto finger_1_sphere_jacobian = (finger_points_1[0] ^ (j_1_1 + j_1_2) ^ (finger_points_1[2] ^ finger_points_1[3]).evaluate()).normalized();

        auto j_2_1 = finger_2_jacobian_1 * (Point<T>() * finger_motors_1[2].reverse()).evaluate();
        auto j_2_2 = (finger_motors_1[2] * Point<T>()).evaluate() * finger_2_jacobian_1.reverse();
        auto finger_2_sphere_jacobian = ((finger_points_1[0] ^ finger_points_1[1]).evaluate() ^ (j_2_1 + j_2_2) ^ finger_points_1[3]).normalized();

        auto j_3_1 = finger_3_jacobian_1 * (Point<T>() * finger_motors_1[3].reverse()).evaluate();
        auto j_3_2 = (finger_motors_1[3] * Point<T>()).evaluate() * finger_3_jacobian_1.reverse();
        auto finger_3_sphere_jacobian = ((finger_points_1[0] ^ finger_points_1[1] ^ finger_points_1[2]).evaluate() ^ (j_3_1 + j_3_2)).normalized();

        MultivectorMatrix<T, Sphere, 1, 16> sphere_jacobian;

        for (unsigned i = 0; i < 4; ++i)
        {
            sphere_jacobian.setCoefficient(0, i, Sphere<T>(finger_0_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 4, Sphere<T>(finger_1_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 8, Sphere<T>(finger_2_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 12, Sphere<T>(finger_3_sphere_jacobian.getCoefficient(0, i)));
        }

        return sphere_jacobian;
    }

    //

}  // namespace gafro