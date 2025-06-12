// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/System.hpp>

namespace gafro
{

    template <class T, int dof>
    class Quadruped : private System<T>
    {
      public:
        Quadruped() = delete;

        Quadruped(const Quadruped &other) = delete;

        System<T> &operator=(System<T> &&other) = delete;

        Quadruped(Quadruped &&other);

        Quadruped &operator=(Quadruped &&other);

        Quadruped(System<T> &&system, const std::array<std::string, 4> &foot_tip_names);

        virtual ~Quadruped();

        //

        System<T> &getSystem();

        const System<T> &getSystem() const;

        //

        Motor<T> getFootMotor(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, Motor, 1, 4> getFootMotors(const Eigen::Vector<T, 4 * dof> &position) const;

        MultivectorMatrix<T, Point, 1, 4> getFootPoints(const Eigen::Vector<T, 4 * dof> &position) const;

        Sphere<T> getFootSphere(const Eigen::Vector<T, 4 * dof> &position) const
            requires(4 == 4);

        MultivectorMatrix<T, Sphere, 1, 4 * dof> getFootSphereJacobian(const Eigen::Vector<T, 4 * dof> &position) const
            requires(4 == 4);

        MultivectorMatrix<T, Motor, 1, dof> getFootAnalyticJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getFootGeometricJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getFootGeometricJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position,
                                                                              const Motor<T> &motor) const;

        MultivectorMatrix<T, Motor, 1, 4 * dof> getAnalyticJacobian(const Eigen::Vector<T, 4 * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> getGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> getGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position, const Motor<T> &motor) const;

        Motor<T> getMeanMotor(const Eigen::Vector<T, 4 * dof> &position) const;

        MultivectorMatrix<T, Motor, 1, 4 * dof> getMeanMotorAnalyticJacobian(const Eigen::Vector<T, 4 * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, 4 * dof> getMeanMotorGeometricJacobian(const Eigen::Vector<T, 4 * dof> &position) const;

        //

      protected:
      private:
        std::array<std::string, 4> foot_tip_names_;
    };

}  // namespace gafro

#include <gafro/robot/Quadruped.hxx>