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
#include <gafro/robot/Quadruped.hpp>

namespace gafro
{

    template <class T, int dof>
    Quadruped<T, dof>::Quadruped(Quadruped &&other)
    {
        *this = std::move(other);
    }

    template <class T, int dof>
    Quadruped<T, dof> &Quadruped<T, dof>::operator=(Quadruped &&other)
    {
        this->System<T>::operator=(std::move(other));
        foot_tip_names_ = std::move(other.foot_tip_names_);
    }

    template <class T, int dof>
    Quadruped<T, dof>::Quadruped(System<T> &&system, const std::array<std::string, 4> &foot_tip_names)
      : System<T>(std::move(system)), foot_tip_names_(foot_tip_names)
    {
        for (int j = 0; j < 4; ++j)
        {
            this->createKinematicChain(foot_tip_names[j]);
        }
    }

    template <class T, int dof>
    Quadruped<T, dof>::~Quadruped() = default;

    //

    template <class T, int dof>
    System<T> &Quadruped<T, dof>::getSystem()
    {
        return *this;
    }

    template <class T, int dof>
    const System<T> &Quadruped<T, dof>::getSystem() const
    {
        return *this;
    }

    //

    template <class T, int dof>
    Motor<T> Quadruped<T, dof>::getFootMotor(const unsigned &id, const Eigen::Vector<T, dof> &position) const
    {
        assert(id < 4);

        return this->computeKinematicChainMotor(foot_tip_names_[id], position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, 4> Quadruped<T, dof>::getFootMotors(const Eigen::Vector<T, 4 * dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, 4> motors;

        for (int j = 0; j < 4; ++j)
        {
            motors.setCoefficient(0, j, getFootMotor(j, position.middleRows(j * dof, dof)));
        }

        return motors;
    }

    template <class T, int dof>
    MultivectorMatrix<T, Point, 1, 4> Quadruped<T, dof>::getFootPoints(const Eigen::Vector<T, 4 * dof> &position) const
    {
        MultivectorMatrix<T, Point, 1, 4> points;

        for (int j = 0; j < 4; ++j)
        {
            points.setCoefficient(0, j, getFootMotor(j, position.middleRows(j * dof, dof)).apply(Point<T>()));
        }

        return points;
    }

    template <class T, int dof>
    Sphere<T> Quadruped<T, dof>::getFootSphere(const Eigen::Vector<T, 4 * dof> &position) const
        requires(4 == 4)
    {
        MultivectorMatrix<T, Point, 1, 4> points = getFootPoints(position);

        return Sphere<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2), points.getCoefficient(0, 3));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> Quadruped<T, dof>::getFootAnalyticJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position) const
    {
        assert(id < 4);

        return this->computeKinematicChainAnalyticJacobian(foot_tip_names_[id], position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Quadruped<T, dof>::getFootGeometricJacobian(const unsigned &id,
                                                                                             const Eigen::Vector<T, dof> &position) const
    {
        assert(id < 4);

        return this->computeKinematicChainGeometricJacobian(foot_tip_names_[id], position);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> Quadruped<T, dof>::getFootGeometricJacobian(const unsigned &id,
                                                                                             const Eigen::Vector<T, dof> &position,
                                                                                             const Motor<T> &motor) const
    {
        return getFootGeometricJacobian(id, position).transform(motor.reverse());
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, 4 * dof> Quadruped<T, dof>::getAnalyticJacobian(const Eigen::Vector<T, 4 * dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, 4 * dof> jacobian;

        for (int j = 0; j < 4; ++j)
        {
            MultivectorMatrix<T, Motor, 1, dof> foot_jacobian = getFootAnalyticJacobian(j, position.middleRows(j * dof, dof));

            for (int k = 0; k < dof; ++k)
            {
                jacobian.setCoefficient(0, j * dof + k, foot_jacobian.getCoefficient(0, k));
            }
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> Quadruped<T, dof>::getGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> jacobian;

        for (int j = 0; j < 4; ++j)
        {
            MultivectorMatrix<T, MotorGenerator, 1, dof> foot_jacobian = getFootGeometricJacobian(j, position.middleRows(j * dof, dof));

            for (int k = 0; k < dof; ++k)
            {
                jacobian.setCoefficient(0, j * dof + k, foot_jacobian.getCoefficient(0, k));
            }
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> Quadruped<T, dof>::getGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position,
                                                                                             const Motor<T> &motor) const
    {
        return getGeometricJacobian(position).transform(motor.reverse());
    }

    template <class T, int dof>
    Motor<T> Quadruped<T, dof>::getMeanMotor(const Eigen::Vector<T, 4 * dof> &position) const
    {
        typename Motor<T>::Generator mean_bivector;

        for (const auto &m : getFootMotors(position).asVector())
        {
            mean_bivector += m.log().evaluate();
        }

        return Motor<T>::exp(mean_bivector / static_cast<T>(4));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, 4 * dof> Quadruped<T, dof>::getMeanMotorAnalyticJacobian(const Eigen::Vector<T, 4 * dof> &position) const
    {
        Motor<T> mean_motor = getMeanMotor(position);

        Eigen::Matrix<T, 8, 6> exp_jacobian = Motor<T>::Exponential::getJacobian(mean_motor.log());

        MultivectorMatrix<T, Motor, 1, 4 * dof> jacobian;

        for (int j = 0; j < 4; ++j)
        {
            MultivectorMatrix<T, Motor, 1, dof> foot_jacobian = getFootAnalyticJacobian(j, position.middleRows(j * dof, dof));

            Eigen::Matrix<T, 6, 8> log_jacobian = Motor<T>::Logarithm::getJacobian(getFootMotor(j, position.middleRows(j * dof, dof)));

            for (int k = 0; k < dof; ++k)
            {
                typename Motor<T>::Parameters parameters =
                  exp_jacobian * log_jacobian / static_cast<double>(4) * foot_jacobian.getCoefficient(0, k).vector();

                jacobian.setCoefficient(0, j * dof + k, Motor<T>(parameters));
            }
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> Quadruped<T, dof>::getMeanMotorGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> jacobian;

        MultivectorMatrix<T, Motor, 1, 4 * dof> analytic_jacobian = getMeanMotorAnalyticJacobian(position);

        for (int j = 0; j < 4 * dof; ++j)
        {
            jacobian.setCoefficient(0, j, -2.0 * analytic_jacobian.getCoefficient(0, j) * getMeanMotor(position).reverse());
        }

        return jacobian;
    }
    template <class T, int dof>
    MultivectorMatrix<T, Sphere, 1, 4 * dof> Quadruped<T, dof>::getFootSphereJacobian(const Eigen::Vector<T, 4 * dof> &position) const
        requires(4 == 4)
    {
        auto foot_motors_1 = getFootMotors(position).asVector();
        auto foot_points_1 = getFootPoints(position).asVector();
        auto foot_0_jacobian_1 = getFootAnalyticJacobian(0, position.middleRows(0, 4));
        auto foot_1_jacobian_1 = getFootAnalyticJacobian(1, position.middleRows(4, 4));
        auto foot_2_jacobian_1 = getFootAnalyticJacobian(2, position.middleRows(8, 4));
        auto foot_3_jacobian_1 = getFootAnalyticJacobian(3, position.middleRows(12, 4));

        auto j_0_1 = foot_0_jacobian_1 * (Point<T>() * foot_motors_1[0].reverse()).evaluate();
        auto j_0_2 = (foot_motors_1[0] * Point<T>()).evaluate() * foot_0_jacobian_1.reverse();
        auto foot_0_sphere_jacobian = ((j_0_1 + j_0_2) ^ (foot_points_1[1] ^ foot_points_1[2] ^ foot_points_1[3]).evaluate()).normalized();

        auto j_1_1 = foot_1_jacobian_1 * (Point<T>() * foot_motors_1[1].reverse()).evaluate();
        auto j_1_2 = (foot_motors_1[1] * Point<T>()).evaluate() * foot_1_jacobian_1.reverse();
        auto foot_1_sphere_jacobian = (foot_points_1[0] ^ (j_1_1 + j_1_2) ^ (foot_points_1[2] ^ foot_points_1[3]).evaluate()).normalized();

        auto j_2_1 = foot_2_jacobian_1 * (Point<T>() * foot_motors_1[2].reverse()).evaluate();
        auto j_2_2 = (foot_motors_1[2] * Point<T>()).evaluate() * foot_2_jacobian_1.reverse();
        auto foot_2_sphere_jacobian = ((foot_points_1[0] ^ foot_points_1[1]).evaluate() ^ (j_2_1 + j_2_2) ^ foot_points_1[3]).normalized();

        auto j_3_1 = foot_3_jacobian_1 * (Point<T>() * foot_motors_1[3].reverse()).evaluate();
        auto j_3_2 = (foot_motors_1[3] * Point<T>()).evaluate() * foot_3_jacobian_1.reverse();
        auto foot_3_sphere_jacobian = ((foot_points_1[0] ^ foot_points_1[1] ^ foot_points_1[2]).evaluate() ^ (j_3_1 + j_3_2)).normalized();

        MultivectorMatrix<T, Sphere, 1, 16> sphere_jacobian;

        for (unsigned i = 0; i < 4; ++i)
        {
            sphere_jacobian.setCoefficient(0, i, Sphere<T>(foot_0_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 4, Sphere<T>(foot_1_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 8, Sphere<T>(foot_2_sphere_jacobian.getCoefficient(0, i)));
            sphere_jacobian.setCoefficient(0, i + 12, Sphere<T>(foot_3_sphere_jacobian.getCoefficient(0, i)));
        }

        return sphere_jacobian;
    }

    //

}  // namespace gafro