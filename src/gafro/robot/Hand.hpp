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

    template <class T, int n_fingers, int dof>
    class Hand : private System<T>
    {
      public:
        Hand() = delete;

        Hand(const Hand &other) = delete;

        System<T> &operator=(System<T> &&hand) = delete;

        Hand(Hand &&hand);

        Hand &operator=(Hand &&hand);

        Hand(System<T> &&system, const std::array<std::string, n_fingers> &finger_tip_names);

        virtual ~Hand();

        //

        System<T> &getSystem();

        const System<T> &getSystem() const;

        //

        Motor<T> getFingerMotor(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, Motor, 1, n_fingers> getFingerMotors(const Eigen::Vector<T, n_fingers * dof> &position) const;

        MultivectorMatrix<T, Point, 1, n_fingers> getFingerPoints(const Eigen::Vector<T, n_fingers * dof> &position) const;

        Sphere<T> getFingerSphere(const Eigen::Vector<T, n_fingers * dof> &position) const
            requires(n_fingers == 4);

        MultivectorMatrix<T, Sphere, 1, n_fingers * dof> getFingerSphereJacobian(const Eigen::Vector<T, n_fingers * dof> &position) const
            requires(n_fingers == 4);

        MultivectorMatrix<T, Motor, 1, dof> getFingerAnalyticJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getFingerGeometricJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getFingerGeometricJacobian(const unsigned &id, const Eigen::Vector<T, dof> &position,
                                                                                const Motor<T> &motor) const;

        MultivectorMatrix<T, Motor, 1, n_fingers * dof> getAnalyticJacobian(const Eigen::Vector<T, n_fingers * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> getGeometricJacobian(const Eigen::Vector<T, n_fingers * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> getGeometricJacobian(const Eigen::Vector<T, n_fingers * dof> &position,
                                                                                      const Motor<T> &motor) const;

        Motor<T> getMeanMotor(const Eigen::Vector<T, n_fingers * dof> &position) const;

        MultivectorMatrix<T, Motor, 1, n_fingers * dof> getMeanMotorAnalyticJacobian(const Eigen::Vector<T, n_fingers * dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, n_fingers * dof> getMeanMotorGeometricJacobian(
          const Eigen::Vector<T, n_fingers * dof> &position) const;

        //

      protected:
      private:
        std::array<std::string, n_fingers> finger_tip_names_;
    };

}  // namespace gafro

#include <gafro/robot/Hand.hxx>