#pragma once

namespace gafro
{

    class CooperativeDualTaskSpace
    {
      public:
        CooperativeDualTaskSpace();

        virtual ~CooperativeDualTaskSpace();

        Motor<T> getAbsoluteMotor(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        Motor<T> getAbsoluteMotor(const Eigen::Vector<T, dof> &positions) const;

        Motor<T> getRelativeMotor(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        Motor<T> getRelativeMotor(const Eigen::Vector<T, dof> &positions) const;

        PointPair<T> getEEPointPair(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        MultivectorMatrix<T, Motor, 1, dof> getRelativeAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const;

        MultivectorMatrix<T, Motor, 1, dof> getRelativeAnalyticJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                        const Eigen::Vector<T, dof / 2> &position_second) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobian(const Eigen::Vector<T, dof> &positions,
                                                                                  const Motor<T>              &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                  const Eigen::Vector<T, dof / 2> &position_second,
                                                                                  const Motor<T>                  &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobianTimeDerivative(const Eigen::Vector<T, dof> &positions,
                                                                                                const Eigen::Vector<T, dof> &velocities,
                                                                                                const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobianTimeDerivative(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                                const Eigen::Vector<T, dof / 2> &position_second,
                                                                                                const Eigen::Vector<T, dof / 2> &velocity_first,
                                                                                                const Eigen::Vector<T, dof / 2> &velocity_second,
                                                                                                const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, Motor, 1, dof> getAbsoluteAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const;

        MultivectorMatrix<T, Motor, 1, dof> getAbsoluteAnalyticJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                        const Eigen::Vector<T, dof / 2> &position_second) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getAbsoluteGeometricJacobian(const Eigen::Vector<T, dof> &positions,
                                                                                  const Motor<T>              &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getAbsoluteGeometricJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                  const Eigen::Vector<T, dof / 2> &position_second,
                                                                                  const Motor<T>                  &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getAbsoluteGeometricJacobianTimeDerivative(const Eigen::Vector<T, dof> &positions,
                                                                                                const Eigen::Vector<T, dof> &velocities,
                                                                                                const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getAbsoluteGeometricJacobianTimeDerivative(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                                const Eigen::Vector<T, dof / 2> &position_second,
                                                                                                const Eigen::Vector<T, dof / 2> &velocity_first,
                                                                                                const Eigen::Vector<T, dof / 2> &velocity_second,
                                                                                                const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, PointPair, 1, dof> getPointPairJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                     const Eigen::Vector<T, dof / 2> &position_second) const;

        //

        Eigen::Matrix<T, 6, 6> getRelativeVelocityManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getRelativeVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                 const Eigen::Vector<T, dof / 2> &position_second) const;

        Eigen::Matrix<T, 6, 6> getRelativeForceManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getRelativeForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                              const Eigen::Vector<T, dof / 2> &position_second) const;

        Eigen::Matrix<T, 6, 6> getRelativeDynamicManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getRelativeDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                const Eigen::Vector<T, dof / 2> &position_second) const;

        Eigen::Matrix<T, 6, 6> getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                 const Eigen::Vector<T, dof / 2> &position_second) const;

        Eigen::Matrix<T, 6, 6> getAbsoluteForceManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getAbsoluteForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                              const Eigen::Vector<T, dof / 2> &position_second) const;

        Eigen::Matrix<T, 6, 6> getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof> &positions) const;
        Eigen::Matrix<T, 6, 6> getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                const Eigen::Vector<T, dof / 2> &position_second) const;

      protected:
      private:
    };

}  // namespace gafro