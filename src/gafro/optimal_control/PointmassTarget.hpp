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

#include <gafro/algebra.hpp>
#include <gafro/robot.hpp>

namespace gafro
{

    template <class T, template <class Type> class Target>
    class SingleManipulatorTarget
    {
      public:
        using Result = typename OuterProduct<Target<T>, Point<T>>::Type;

        using VectorX = Eigen::Matrix<T, 6, 1>;
        using MatrixXX = Eigen::Matrix<T, 6, 6>;

        SingleManipulatorTarget(const SingleManipulatorTarget &other) : target_(other.target_) {}

        SingleManipulatorTarget(const Target<T> &target) : target_(target) {}

        T getValue(const VectorX &x) const
        {
            return getError(x).squaredNorm();
        }

        VectorX getGradient(const VectorX &x) const
        {
            Eigen::Matrix<T, Result::size, 6> jacobian = getJacobian(x);

            return jacobian.transpose() * getError(x);
        }

        void getGradientAndHessian(const VectorX &x, VectorX &gradient, MatrixXX &hessian) const
        {
            Eigen::Matrix<T, Result::size, 6> jacobian = getJacobian(x);

            gradient = jacobian.transpose() * getError(x);
            hessian = jacobian.transpose() * jacobian;
        }

        Eigen::Matrix<T, Result::size, 1> getError(const VectorX &x) const
        {
            return Result(target_ ^ Motor<T>::exp(Motor<T>::Generator(x)).apply(Point<T>())).vector();
        }

        Eigen::Matrix<T, Result::size, 6> getJacobian(const VectorX &x) const
        {
            Motor<T> motor = arm_.getEEMotor(x);
            MultivectorMatrix<Motor<T>, 1, 6> jacobian_ee = arm_.getEEAnalyticJacobian(x);

            Eigen::Matrix<T, Result::size, 6> jacobian;

            for (unsigned i = 0; i < 6; ++i)
            {
                jacobian.col(i) = 2.0 * Result(target_ ^ (jacobian_ee[i] * tool_ * motor.reverse() +  //
                                                          motor * tool_ * jacobian_ee[i].reverse()))
                                          .vector();
            }

            return jacobian;
        }

      private:
        Target<T> target_;

        FrankaEmikaRobot<T> arm_;
    };

}  // namespace gafro