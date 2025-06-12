// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>
#include <gafro/robot.hpp>

namespace gafro
{

    template <class T, int dof, template <class Type> class Tool, template <class Type> class Target>
    class SingleManipulatorTarget
    {
      public:
        using Result = typename OuterProduct<Target<T>, SandwichProduct<Tool<T>, Motor<T>>>::Type;

        using VectorX = Eigen::Matrix<T, dof, 1>;
        using MatrixXX = Eigen::Matrix<T, dof, dof>;

        SingleManipulatorTarget(const Manipulator<T, dof> *arm, const Tool<T> &tool, const Target<T> &target)
          : arm_(arm), tool_(tool), target_(target)
        {}

        T getValue(const VectorX &x) const
        {
            return getError(x).squaredNorm();
        }

        VectorX getGradient(const VectorX &x) const
        {
            Eigen::Matrix<T, Result::size, dof> jacobian = getJacobian(x);

            return jacobian.transpose() * getError(x);
        }

        void getGradientAndHessian(const VectorX &x, VectorX &gradient, MatrixXX &hessian) const
        {
            Eigen::Matrix<T, Result::size, dof> jacobian = getJacobian(x);

            gradient = jacobian.transpose() * getError(x);
            hessian = jacobian.transpose() * jacobian;
        }

        Eigen::Matrix<T, Result::size, 1> getError(const VectorX &x) const
        {
            return Result(target_ ^ arm_->getEEMotor(x).apply(tool_)).vector();
        }

        Eigen::Matrix<T, Result::size, dof> getJacobian(const VectorX &x) const
        {
            Motor<T> motor = arm_->getEEMotor(x);
            MultivectorMatrix<T, Motor, 1, dof> jacobian_ee = arm_->getEEAnalyticJacobian(x);

            Eigen::Matrix<T, Result::size, dof> jacobian;

            for (unsigned i = 0; i < dof; ++i)
            {
                jacobian.col(i) = 2.0 * Result(target_ ^ (jacobian_ee.getCoefficient(0, i) * tool_ * motor.reverse() +  //
                                                          motor * tool_ * jacobian_ee.getCoefficient(0, i).reverse()))
                                          .vector();
            }

            return jacobian;
        }

      private:
        const Manipulator<T, dof> *arm_;

        Tool<T> tool_;

        Target<T> target_;
    };

}  // namespace gafro