// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Manipulator.hpp>

namespace gafro
{

    template <class T, int dof>
    class DualManipulator : public System<T>
    {
      public:
        DualManipulator() = delete;

        DualManipulator(const DualManipulator &other) = delete;

        System<T> &operator=(System<T> &&manipulator) = delete;

        DualManipulator(DualManipulator &&manipulator);

        DualManipulator &operator=(DualManipulator &&manipulator);

        DualManipulator(System<T> &&system, const std::string &first_ee_joint_name, const std::string &second_ee_joint_name);

        DualManipulator(const Manipulator<T, dof / 2> &manipulator, const Motor<T> &first_base_motor, const Motor<T> &second_base_motor);

        virtual ~DualManipulator();

        //

        const System<T> &getSystem() const;

        //

        Motor<T> getFirstBaseMotor() const;

        Motor<T> getSecondBaseMotor() const;

        //

        const KinematicChain<T> *getFirstKinematicChain() const;

        const KinematicChain<T> *getSecondKinematicChain() const;

        //

        Motor<T> getFirstEEMotor(const Eigen::Vector<T, dof / 2> &position) const;

        Motor<T> getSecondEEMotor(const Eigen::Vector<T, dof / 2> &position) const;

        Motor<T> getAbsoluteMotor(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        Motor<T> getAbsoluteMotor(const Eigen::Vector<T, dof> &positions) const;

        Motor<T> getRelativeMotor(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        Motor<T> getRelativeMotor(const Eigen::Vector<T, dof> &positions) const;

        PointPair<T> getEEPointPair(const Eigen::Vector<T, dof / 2> &position_first, const Eigen::Vector<T, dof / 2> &position_second) const;

        MultivectorMatrix<T, Motor, 1, dof> getRelativeAnalyticJacobian(const Eigen::Vector<T, dof> &positions) const;

        MultivectorMatrix<T, Motor, 1, dof> getRelativeAnalyticJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                        const Eigen::Vector<T, dof / 2> &position_second) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobian(const Eigen::Vector<T, dof> &positions,
                                                                                  const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getRelativeGeometricJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                  const Eigen::Vector<T, dof / 2> &position_second,
                                                                                  const Motor<T> &reference = Motor<T>()) const;

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
                                                                                  const Motor<T> &reference = Motor<T>()) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getAbsoluteGeometricJacobian(const Eigen::Vector<T, dof / 2> &position_first,
                                                                                  const Eigen::Vector<T, dof / 2> &position_second,
                                                                                  const Motor<T> &reference = Motor<T>()) const;

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

      public:
        Eigen::Matrix<T, dof, 1> getJointTorques(const Eigen::Vector<T, dof> &positions, const Eigen::Vector<T, dof> &velocities,
                                                 const Eigen::Vector<T, dof> &accelerations, const T &gravity);

      protected:
      private:
        std::string first_ee_joint_name_;

        std::string second_ee_joint_name_;
    };

}  // namespace gafro

#include <gafro/robot/DualManipulator.hxx>