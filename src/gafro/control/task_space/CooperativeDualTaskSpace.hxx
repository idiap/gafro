// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/control/task_space/CooperativeDualTaskSpace.hpp>
#include <gafro/robot/FixedJoint.hpp>
#include <gafro/robot/PrismaticJoint.hpp>
#include <gafro/robot/RevoluteJoint.hpp>

namespace gafro
{

    template <class T, int dof>
    CooperativeDualTaskSpace<T, dof>::CooperativeDualTaskSpace(System<T> *system, const std::array<std::string, 2> &kinematic_chains)
      : CooperativeTaskSpace<T, 2, dof>(system, kinematic_chains)
    {
        first_ee_joint_name_  = kinematic_chains[0];
        second_ee_joint_name_ = kinematic_chains[1];
    }

    template <class T, int dof>
    CooperativeDualTaskSpace<T, dof>::~CooperativeDualTaskSpace() = default;

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getFirstBaseMotor() const
    {
        return this->getLink("base_link")->getChildJoints()[0]->getMotor(0.0);
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getSecondBaseMotor() const
    {
        return this->getLink("base_link")->getChildJoints()[1]->getMotor(0.0);
    }

    //

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getFirstEEMotor(const Eigen::Vector<T, dof / 2> &position) const
    {
        return this->getSystem()->computeKinematicChainMotor(first_ee_joint_name_, position);
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getSecondEEMotor(const Eigen::Vector<T, dof / 2> &position) const
    {
        return this->getSystem()->computeKinematicChainMotor(second_ee_joint_name_, position);
    }

    template <class T, int dof>
    const KinematicChain<T> *CooperativeDualTaskSpace<T, dof>::getFirstKinematicChain() const
    {
        return this->getSystem()->getKinematicChain(first_ee_joint_name_);
    }

    template <class T, int dof>
    const KinematicChain<T> *CooperativeDualTaskSpace<T, dof>::getSecondKinematicChain() const
    {
        return this->getSystem()->getKinematicChain(second_ee_joint_name_);
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getAbsoluteMotor(const Eigen::Vector<T, dof> &positions) const
    {
        return this->getAbsoluteMotor(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getAbsoluteMotor(const Eigen::Vector<T, dof / 2> &position_first,
                                                                const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getSecondEEMotor(position_second) *
               Motor<T>::exp(Scalar<T>(TypeTraits<T>::Value(0.5)) * getRelativeMotor(position_first, position_second).log());
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getRelativeMotor(const Eigen::Vector<T, dof> &positions) const
    {
        return this->getRelativeMotor(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Motor<T> CooperativeDualTaskSpace<T, dof>::getRelativeMotor(const Eigen::Vector<T, dof / 2> &position_first,
                                                                const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getSecondEEMotor(position_second).reverse() * getFirstEEMotor(position_first);
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeAnalyticJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeAnalyticJacobian(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T>                                first_motor  = getFirstEEMotor(position_first);
        Motor<T>                                second_motor = getSecondEEMotor(position_second);
        MultivectorMatrix<T, Motor, 1, dof / 2> first_jacobian =
          this->getSystem()->computeKinematicChainAnalyticJacobian(first_ee_joint_name_, position_first);
        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian =
          this->getSystem()->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, Motor, 1, dof> jacobian;

        for (unsigned j = 0; j < dof / 2; ++j)
        {
            jacobian.setCoefficient(0, j, second_motor.reverse() * first_jacobian.getCoefficient(0, j));
            jacobian.setCoefficient(0, j + dof / 2, Motor<T>(second_jacobian.getCoefficient(0, j)).reverse() * first_motor);
        }

        return jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeGeometricJacobian(
      const Eigen::Vector<T, dof> &positions,
      const Motor<T>              &reference) const
    {
        return getRelativeGeometricJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeGeometricJacobian(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second,
      const Motor<T>                  &reference) const
    {
        Motor<T>                                     relative_motor             = getRelativeMotor(position_first, position_second);
        MultivectorMatrix<T, Motor, 1, dof>          relative_analytic_jacobian = getRelativeAnalyticJacobian(position_first, position_second);
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian;

        for (int j = 0; j < dof; ++j)
        {
            relative_geometric_jacobian.setCoefficient(
              0,
              j,
              reference.reverse() *
                (Scalar<T>(TypeTraits<T>::Value(-2.0)) * relative_analytic_jacobian.getCoefficient(0, j) * relative_motor.reverse()) * reference);
        }

        return relative_geometric_jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteGeometricJacobian(
      const Eigen::Vector<T, dof> &positions,
      const Motor<T>              &reference) const
    {
        return getAbsoluteGeometricJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteGeometricJacobian(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second,
      const Motor<T>                  &reference) const
    {
        Motor<T>                            absolute_motor             = getAbsoluteMotor(position_first, position_second);
        MultivectorMatrix<T, Motor, 1, dof> absolute_analytic_jacobian = getAbsoluteAnalyticJacobian(position_first, position_second);

        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian;

        for (int j = 0; j < dof; ++j)
        {
            absolute_geometric_jacobian.setCoefficient(
              0,
              j,
              reference.reverse() *
                (Scalar<T>(TypeTraits<T>::Value(-2.0)) * absolute_analytic_jacobian.getCoefficient(0, j) * absolute_motor.reverse()) * reference);
        }

        return absolute_geometric_jacobian;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof> &positions,
      const Eigen::Vector<T, dof> &velocities,
      const Motor<T>              &reference) const
    {
        return getRelativeGeometricJacobianTimeDerivative(
          positions.topRows(dof / 2), positions.bottomRows(dof / 2), velocities.topRows(dof / 2), velocities.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getRelativeGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second,
      const Eigen::Vector<T, dof / 2> &velocity_first,
      const Eigen::Vector<T, dof / 2> &velocity_second,
      const Motor<T>                  &reference) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian = getRelativeGeometricJacobian(position_first, position_second);
        Eigen::Matrix<T, 6, dof>                     relative_geometric_jacobian_matrix = relative_geometric_jacobian.embed();
        MultivectorMatrix<T, MotorGenerator, 1, dof> relative_geometric_jacobian_time_derivative;

        for (int j = 0; j < dof / 2; ++j)
        {
            relative_geometric_jacobian_time_derivative.setCoefficient(
              0,
              j,
              reference.reverse() *
                (relative_geometric_jacobian.getCoefficient(0, j).commute(
                  Twist<T>(relative_geometric_jacobian_matrix.block(0, 0, 6, j + 1) * velocity_first.topRows(j + 1)))) *
                reference);

            relative_geometric_jacobian_time_derivative.setCoefficient(
              0,
              dof / 2 + j,
              reference.reverse() *
                (relative_geometric_jacobian.getCoefficient(0, dof / 2 + j)
                   .commute(Twist<T>(relative_geometric_jacobian_matrix.block(0, dof / 2, 6, j + 1) * velocity_second.topRows(j + 1)))) *
                reference);
        }

        return relative_geometric_jacobian_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof> &positions,
      const Eigen::Vector<T, dof> &velocities,
      const Motor<T>              &reference) const
    {
        return getAbsoluteGeometricJacobianTimeDerivative(
          positions.topRows(dof / 2), positions.bottomRows(dof / 2), velocities.topRows(dof / 2), velocities.bottomRows(dof / 2), reference);
    }

    template <class T, int dof>
    MultivectorMatrix<T, MotorGenerator, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteGeometricJacobianTimeDerivative(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second,
      const Eigen::Vector<T, dof / 2> &velocity_first,
      const Eigen::Vector<T, dof / 2> &velocity_second,
      const Motor<T>                  &reference) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian = getAbsoluteGeometricJacobian(position_first, position_second);
        Eigen::Matrix<T, 6, dof>                     absolute_geometric_jacobian_matrix = absolute_geometric_jacobian.embed();
        MultivectorMatrix<T, MotorGenerator, 1, dof> absolute_geometric_jacobian_time_derivative;

        for (int j = 0; j < dof / 2; ++j)
        {
            absolute_geometric_jacobian_time_derivative.setCoefficient(
              0,
              j,
              reference.reverse() *
                (absolute_geometric_jacobian.getCoefficient(0, j).commute(
                  Twist<T>(absolute_geometric_jacobian_matrix.block(0, 0, 6, j + 1) * velocity_first.topRows(j + 1)))) *
                reference);

            absolute_geometric_jacobian_time_derivative.setCoefficient(
              0,
              dof / 2 + j,
              reference.reverse() *
                (absolute_geometric_jacobian.getCoefficient(0, dof / 2 + j)
                   .commute(Twist<T>(absolute_geometric_jacobian_matrix.block(0, dof / 2, 6, j + 1) * velocity_second.topRows(j + 1)))) *
                reference);
        }

        return absolute_geometric_jacobian_time_derivative;
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteAnalyticJacobian(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    MultivectorMatrix<T, Motor, 1, dof> CooperativeDualTaskSpace<T, dof>::getAbsoluteAnalyticJacobian(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T>                     relative_motor = getRelativeMotor(position_first, position_second);
        typename Motor<T>::Generator b              = Scalar<T>(TypeTraits<T>::Value(0.5)) * relative_motor.log();
        Motor<T>                     motor          = Motor<T>::exp(b);
        Motor<T>                     second_motor   = getSecondEEMotor(position_second);

        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian =
          this->getSystem()->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, Motor, 1, dof> absolute_jacobian;

        Eigen::Matrix<T, 8, 6>   exp_jacobian               = Motor<T>::Exponential::getJacobian(b);
        Eigen::Matrix<T, 6, 8>   log_jacobian               = Motor<T>::Logarithm::getJacobian(relative_motor);
        Eigen::Matrix<T, 8, dof> relative_analytic_jacobian = getRelativeAnalyticJacobian(position_first, position_second).embed();

        Eigen::Matrix<T, 8, dof> j = TypeTraits<T>::Value(0.5) * exp_jacobian * log_jacobian * relative_analytic_jacobian;

        for (unsigned i = 0; i < dof / 2; ++i)
        {
            absolute_jacobian.setCoefficient(0, i, second_motor * Motor<T>(j.col(i)));
            absolute_jacobian.setCoefficient(
              0, i + dof / 2, second_jacobian.getCoefficient(0, i) * motor + second_motor * Motor<T>(j.col(i + dof / 2)));
        }

        return absolute_jacobian;
    }

    template <class T, int dof>
    PointPair<T> CooperativeDualTaskSpace<T, dof>::getEEPointPair(const Eigen::Vector<T, dof / 2> &position_first,
                                                                  const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getFirstEEMotor(position_first).apply(Point<T>()) ^ getSecondEEMotor(position_second).apply(Point<T>());
    }

    template <class T, int dof>
    MultivectorMatrix<T, PointPair, 1, dof> CooperativeDualTaskSpace<T, dof>::getPointPairJacobian(
      const Eigen::Vector<T, dof / 2> &position_first,
      const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Motor<T> first_motor  = getFirstEEMotor(position_first);
        Motor<T> second_motor = getSecondEEMotor(position_second);

        Point<T> first_point  = first_motor.apply(Point<T>());
        Point<T> second_point = second_motor.apply(Point<T>());

        MultivectorMatrix<T, Motor, 1, dof / 2> first_jacobian =
          this->getSystem()->computeKinematicChainAnalyticJacobian(first_ee_joint_name_, position_first);
        MultivectorMatrix<T, Motor, 1, dof / 2> second_jacobian =
          this->getSystem()->computeKinematicChainAnalyticJacobian(second_ee_joint_name_, position_second);

        MultivectorMatrix<T, PointPair, 1, dof> pointpair_jacobian;

        for (unsigned j = 0; j < dof / 2; ++j)
        {
            pointpair_jacobian.getCoefficient(0, j) =
              ((getFirstBaseMotor() * first_jacobian.getCoefficient(0, j) * E0<T>(TypeTraits<T>::Value(1.0)) * first_motor.reverse()) ^
               second_point) +
              ((first_motor * E0<T>(TypeTraits<T>::Value(1.0)) * Motor<T>(getFirstBaseMotor() * first_jacobian.getCoefficient(0, j)).reverse()) ^
               second_point);

            pointpair_jacobian.getCoefficient(0, j + j) =
              (first_point ^
               (getSecondBaseMotor() * second_jacobian.getCoefficient(0, j) * E0<T>(TypeTraits<T>::Value(1.0)) * second_motor.reverse())) +
              (first_point ^
               (second_motor * E0<T>(TypeTraits<T>::Value(1.0)) * Motor<T>(getSecondBaseMotor() * second_jacobian.getCoefficient(0, j)).reverse()));
        }

        return pointpair_jacobian;
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeVelocityManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeVelocityManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                               const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getRelativeGeometricJacobian(position_first, position_second).embed();

        return jacobian * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeForceManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeForceManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                            const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getRelativeVelocityManipulability(position_first, position_second).inverse();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeDynamicManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getRelativeDynamicManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getRelativeDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                              const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return Eigen::Matrix<T, 6, 6>::Zero();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteVelocityManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteVelocityManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                               const Eigen::Vector<T, dof / 2> &position_second) const
    {
        Eigen::Matrix<T, 6, dof> jacobian = getAbsoluteGeometricJacobian(position_first, position_second).embed();

        return jacobian * jacobian.transpose();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteForceManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteForceManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteForceManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                            const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return getAbsoluteVelocityManipulability(position_first, position_second).inverse();
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof> &positions) const
    {
        return getAbsoluteDynamicManipulability(positions.topRows(dof / 2), positions.bottomRows(dof / 2));
    }

    template <class T, int dof>
    Eigen::Matrix<T, 6, 6> CooperativeDualTaskSpace<T, dof>::getAbsoluteDynamicManipulability(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                              const Eigen::Vector<T, dof / 2> &position_second) const
    {
        return Eigen::Matrix<T, 6, 6>::Zero();
    }

    template <class T, int dof>
    Eigen::Matrix<T, dof, 1> CooperativeDualTaskSpace<T, dof>::getJointTorques(const Eigen::Vector<T, dof> &positions,
                                                                               const Eigen::Vector<T, dof> &velocities,
                                                                               const Eigen::Vector<T, dof> &accelerations,
                                                                               const T                     &gravity) const
    {
        Eigen::Matrix<T, dof, 1> torques;

        torques.topRows(dof / 2) = this->getSystem()->template computeInverseDynamics<dof / 2>(positions.topRows(dof / 2),      //
                                                                                               velocities.topRows(dof / 2),     //
                                                                                               accelerations.topRows(dof / 2),  //
                                                                                               gravity,                         //
                                                                                               Wrench<T>::Zero(),               //
                                                                                               first_ee_joint_name_);

        torques.bottomRows(dof / 2) = this->getSystem()->template computeInverseDynamics<dof / 2>(positions.bottomRows(dof / 2),      //
                                                                                                  velocities.bottomRows(dof / 2),     //
                                                                                                  accelerations.bottomRows(dof / 2),  //
                                                                                                  gravity,                            //
                                                                                                  Wrench<T>::Zero(),                  //
                                                                                                  second_ee_joint_name_);

        return torques;
    }

    template <class T, int dof>
    typename orwell::RobotState<dof>::Vector CooperativeDualTaskSpace<T, dof>::computeInverseDynamics(
      const typename orwell::RobotState<dof>::Vector &position,
      const typename orwell::RobotState<dof>::Vector &velocity,
      const typename orwell::RobotState<dof>::Vector &acceleration) const
    {
        return this->getJointTorques(position, velocity, acceleration, 0.0);
    }

}  // namespace gafro