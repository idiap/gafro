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