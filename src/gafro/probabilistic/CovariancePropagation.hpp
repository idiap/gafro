#pragma once

#include <Eigen/Core>
#include <iostream>

namespace gafro::probabilistic
{

    template <class LeftOperand, class RightOperand, template <class L, class R> class Operation>
    class CovariancePropagation
    {
      public:
        using Result = typename Operation<typename LeftOperand::Mean, typename RightOperand::Mean>::Type;
        constexpr static int size = Result::size;
        using Covariance = Eigen::Matrix<double, size, size>;

        static Covariance propagate(const LeftOperand &left_operand, const RightOperand &right_operand)
        {
            Operation<typename LeftOperand::Mean, typename RightOperand::Mean> operation(left_operand, right_operand);

            auto left_jacobian = operation.getLeftJacobian();
            auto right_jacobian = operation.getRightJacobian();

            return left_jacobian * left_operand.getCovariance() * left_jacobian.transpose() +
                   right_jacobian * right_operand.getCovariance() * right_jacobian.transpose();
        }

      protected:
      private:
    };

}  // namespace gafro::probabilistic