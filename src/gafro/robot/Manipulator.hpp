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

#include <Eigen/Core>
#include <array>
#include <functional>
#include <memory>
#include <vector>
//
#include <gafro/robot/Joint.hpp>
#include <gafro/robot/Link.hpp>

namespace gafro
{
    template <class M, int rows, int cols>
    class MultivectorMatrix;

    template <class T>
    class EndEffector;

    template <class T, int dof>
    class Manipulator
    {
      public:
        using State = Eigen::Matrix<T, dof, 1>;

        Manipulator() = delete;

        Manipulator(const std::array<std::array<T, 3>, dof> &dh_parameters, const std::array<T, 3> &end_effector_link,
                    std::unique_ptr<EndEffector<T>> &&end_effector);

        Manipulator(const std::array<std::array<T, 6>, dof> &parameters, const std::array<int, dof> &axis);

        Manipulator(const Manipulator &other);

        virtual ~Manipulator();

        State getRandomConfiguration() const;

        void setJointLimits(const State &joint_limits_min, const State &joint_limits_max);

        const State &getJointLimitsMin() const;

        const State &getJointLimitsMax() const;

        //

        void setBase(const Motor<T> &base);

        const Motor<T> &getBase() const;

        //

        Link<T> &getLink(const unsigned &index);

        const Link<T> &getLink(const unsigned &index) const;

        const std::array<Link<T>, dof> &getLinks() const;

        Joint<T> &getJoint(const unsigned &index);

        const Joint<T> &getJoint(const unsigned &index) const;

        const std::array<Joint<T>, dof> &getJoints() const;

        //

        Motor<T> getJointMotor(const unsigned &id, const State &position) const;

        const std::array<Motor<T>, dof> &getJointMotors(const State &position) const;

        //

        Motor<T> getEEMotor(const State &position) const;

        Eigen::Matrix<T, 8, dof> getJointJacobian(const unsigned &id, const State &position, const Motor<T> &base = Motor<T>()) const;

        //

        const Motor<T> &getEndEffectorLink() const;

        MultivectorMatrix<Motor<T>, 1, dof> getEEAnalyticJacobian(const State &position) const;

        MultivectorMatrix<typename Motor<T>::Generator, 1, dof> getEEGeometricJacobian(const State &position) const;

        MultivectorMatrix<Motor<T>, dof, dof> getFullGeometricJacobian(const State &position) const;

        MultivectorMatrix<Motor<T>, 1, dof> getEEGeometricJacobianTimeDerivative(const State &position, const State &velocity) const;

        MultivectorMatrix<Motor<T>, dof, dof> getFullGeometricJacobianTimeDerivative(const State &position, const State &velocity) const;

        MultivectorMatrix<Scalar<T>, dof, dof> getMassMatrix(const State &position) const;

        MultivectorMatrix<Scalar<T>, dof, dof> getInertiaMatrix(const State &position) const;

        MultivectorMatrix<Scalar<T>, dof, 1> getInertiaMatrixTimeDerivative(const State &position, const State &velocity) const;

        MultivectorMatrix<Scalar<T>, dof, 1> getGravityVector(const State &position) const;

        MultivectorMatrix<Scalar<T>, dof, 1> getCoriolisVector(const State &position, const State &velocity) const;

        MultivectorMatrix<Point<T>, dof, dof> getLinkCoMs(const State &position) const;

        State getJointTorques(const State &position, const State &velocity, const State &acceleration) const;

        // State getJointAccelerations(const State &position, const State &velocity, const State &torques) const;

        // T getPotentialEnergy(const State &position) const;

        // T getKineticEnergy(const State &position, const State &velocity) const;

        // T getEnergy(const State &position, const State &velocity) const;

        // Eigen::Matrix<T, dof, dof> getGeneralizedMassMatrix(const State &position, const State &velocity) const;

        // Eigen::Matrix<T, dof, dof> getNonlinearities(const State &position, const State &velocity) const;

        MultivectorMatrix<typename InnerProduct<Point<T>, typename Motor<T>::Generator>::Type, dof, dof> getProjectedMassMatrix(
          const State &position) const;

        const EndEffector<T> *getEndEffector() const;

      protected:
      private:
        mutable std::array<Motor<T>, dof> motors_;

        std::array<Joint<T>, dof> joints_;

        std::array<Link<T>, dof> links_;

        Motor<T> base_;

        Motor<T> end_effector_link_;

        std::unique_ptr<EndEffector<T>> end_effector_;

        State joint_limits_min_;

        State joint_limits_max_;

      public:
        using Ptr = std::shared_ptr<Manipulator>;
        using ConstPtr = std::shared_ptr<const Manipulator>;
    };
}  // namespace gafro