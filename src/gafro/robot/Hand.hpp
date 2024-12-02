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

    template <class T, int... fingers>
    class Hand : private System<T>
    {
        constexpr static int n_fingers = sizeof...(fingers);
        constexpr static int dof = (fingers + ...);
        constexpr static std::array<int, n_fingers> finger_dof = { fingers... };

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

        template <int id>
        Motor<T> getFingerMotor(const Eigen::Vector<T, finger_dof[id]> &position) const;

        template <int id>
        MultivectorMatrix<T, Motor, 1, finger_dof[id]> getFingerAnalyticJacobian(const Eigen::Vector<T, finger_dof[id]> &position) const;

        template <int id>
        MultivectorMatrix<T, MotorGenerator, 1, finger_dof[id]> getFingerGeometricJacobian(const Eigen::Vector<T, finger_dof[id]> &position) const;

        template <int id>
        MultivectorMatrix<T, MotorGenerator, 1, finger_dof[id]> getFingerGeometricJacobian(const Eigen::Vector<T, finger_dof[id]> &position,
                                                                                           const Motor<T> &motor) const;

        //

        MultivectorMatrix<T, Motor, 1, n_fingers> getFingerMotors(const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, Point, 1, n_fingers> getFingerPoints(const Eigen::Vector<T, dof> &position) const;

        //

        MultivectorMatrix<T, Motor, 1, dof> getAnalyticJacobian(const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getGeometricJacobian(const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getGeometricJacobian(const Eigen::Vector<T, dof> &position, const Motor<T> &motor) const;

        Motor<T> getMeanMotor(const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, Motor, 1, dof> getMeanMotorAnalyticJacobian(const Eigen::Vector<T, dof> &position) const;

        MultivectorMatrix<T, MotorGenerator, 1, dof> getMeanMotorGeometricJacobian(const Eigen::Vector<T, dof> &position) const;

        //

        Circle<T> getFingerCircle(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 3);

        MultivectorMatrix<T, Circle, 1, dof> getFingerCircleJacobian(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 3);

        Sphere<T> getFingerSphere(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 4);

        MultivectorMatrix<T, Sphere, 1, dof> getFingerSphereJacobian(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 4);

        //

      private:
        template <long unsigned... idx>
        constexpr void forEachFinger(auto &f, const Eigen::Vector<T, dof> &position, std::index_sequence<idx...>) const
        {
            (f.template call<idx>(position), ...);
        }

        template <int idx>
        static auto extract(const Eigen::Vector<T, dof> &position)
        {
            auto p = position.middleRows(idx == 0 ? 0 : std::accumulate(finger_dof.begin(), finger_dof.begin() + idx, 0), finger_dof[idx]);

            return p;
        }

        struct FingerFunctor
        {
            FingerFunctor(const Hand *hand) : hand_(hand) {}

            const Hand *hand_;
        };

        struct FingerMotorFunctor : public FingerFunctor
        {
            MultivectorMatrix<T, Motor, 1, n_fingers> motors;

            using FingerFunctor::hand_;

            template <int idx>
            void call(const Eigen::Vector<T, dof> &position)
            {
                motors.setCoefficient(0, idx, hand_->template getFingerMotor<idx>(Hand::template extract<idx>(position)));
            }
        };

        struct FingerAnalyticJacobianFunctor : public FingerFunctor
        {
            MultivectorMatrix<T, Motor, 1, dof> jacobian;

            using FingerFunctor::hand_;

            template <int idx>
            void call(const Eigen::Vector<T, dof> &position)
            {
                MultivectorMatrix<T, Motor, 1, finger_dof[idx]> finger_jacobian =
                  hand_->template getFingerAnalyticJacobian<idx>(Hand::template extract<idx>(position));

                for (int k = 0; k < finger_dof[idx]; ++k)
                {
                    jacobian.setCoefficient(0, std::accumulate(finger_dof.begin(), finger_dof.begin() + idx, 0) + k,
                                            finger_jacobian.getCoefficient(0, k));
                }
            }
        };

        struct FingerGeometricJacobianFunctor : public FingerFunctor
        {
            MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian;

            using FingerFunctor::hand_;

            template <int idx>
            void call(const Eigen::Vector<T, dof> &position)
            {
                MultivectorMatrix<T, MotorGenerator, 1, finger_dof[idx]> finger_jacobian =
                  hand_->template getFingerGeometricJacobian<idx>(Hand::template extract<idx>(position));

                for (int k = 0; k < finger_dof[idx]; ++k)
                {
                    jacobian.setCoefficient(0, std::accumulate(finger_dof.begin(), finger_dof.begin() + idx, 0) + k,
                                            finger_jacobian.getCoefficient(0, k));
                }
            }
        };

      protected:
      private:
        std::array<std::string, n_fingers> finger_tip_names_;
    };

}  // namespace gafro

#include <gafro/robot/Hand.hxx>