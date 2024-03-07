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

    template <class T, int dof>
    class SingleManipulatorMotorCost
    {
      public:
        using VectorX = Eigen::Matrix<T, dof, 1>;
        using MatrixXX = Eigen::Matrix<T, dof, dof>;

        SingleManipulatorMotorCost(const Manipulator<T, dof> *arm, const Motor<T> &target)  //
          : target_(target), arm_(arm)
        {}

        Eigen::Matrix<T, 6, 1> getError(const VectorX &x) const
        {
            return Motor<T>(target_.reverse() * arm_->getEEMotor(x)).log().evaluate().vector();
        }

        void getGradientAndHessian(const VectorX &state, VectorX &gradient, MatrixXX &hessian) const
        {
            Eigen::Matrix<T, 6, 1> error = getError(state);
            Eigen::Matrix<T, 6, 8> jacobian_log = Motor<T>(target_.reverse() * arm_->getEEMotor(state)).logJacobian();
            MultivectorMatrix<T, Motor, 1, dof> jacobian_ee = arm_->getEEAnalyticJacobian(state);

            for (unsigned i = 0; i < dof; ++i)
            {
                jacobian_ee.getCoefficient(0, i) = target_.reverse() * jacobian_ee.getCoefficient(0, i);
            }

            Eigen::Matrix<T, 6, dof> jacobian = jacobian_log * jacobian_ee.embed();

            gradient = VectorX::Zero();
            hessian = MatrixXX::Zero();

            gradient.block(0, 0, dof, 1) = jacobian.transpose() * error;
            hessian.block(0, 0, dof, dof) = jacobian.transpose() * jacobian;
        }

      private:
        Motor<T> target_;

        const Manipulator<T, dof> *arm_;
    };
}  // namespace gafro