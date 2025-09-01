// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>
//
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

        // Implemented here instead of in 'Hand.hxx' to avoid an issue in older compilers with 'requires'
        Circle<T> getFingerCircle(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 3)
        {
            MultivectorMatrix<T, Point, 1, n_fingers> points = getFingerPoints(position);

            return Circle<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2));
        }

        // Implemented here instead of in 'Hand.hxx' to avoid an issue in older compilers with 'requires'
        MultivectorMatrix<T, Circle, 1, dof> getFingerCircleJacobian(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 3)
        {
            auto finger_motors = getFingerMotors(position).asVector();
            auto finger_points = getFingerPoints(position).asVector();
            auto finger_jacobian = getAnalyticJacobian(position);

            MultivectorMatrix<T, Circle, 1, dof> circle_jacobian;

            for (int k = 0; k < finger_dof[0]; ++k)
            {
                Point<T> j_0_1 = finger_jacobian.getCoefficient(0, k) * Point<T>() * finger_motors[0].reverse();
                Point<T> j_0_2 = finger_motors[0] * Point<T>() * finger_jacobian.getCoefficient(0, k).reverse();

                circle_jacobian.setCoefficient(0, k, (j_0_1 + j_0_2) ^ finger_points[1] ^ finger_points[2]);
            }

            for (int k = finger_dof[0]; k < finger_dof[0] + finger_dof[1]; ++k)
            {
                Point<T> j_1_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[1].reverse()).evaluate();
                Point<T> j_1_2 = (finger_motors[1] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                circle_jacobian.setCoefficient(0, k, finger_points[0] ^ (j_1_1 + j_1_2) ^ finger_points[2]);
            }

            for (int k = finger_dof[0] + finger_dof[1]; k < dof; ++k)
            {
                Point<T> j_2_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[2].reverse()).evaluate();
                Point<T> j_2_2 = (finger_motors[2] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                circle_jacobian.setCoefficient(0, k, finger_points[0] ^ finger_points[1] ^ (j_2_1 + j_2_2));
            }

            return circle_jacobian;
        }

        // Implemented here instead of in 'Hand.hxx' to avoid an issue in older compilers with 'requires'
        Sphere<T> getFingerSphere(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 4)
        {
            MultivectorMatrix<T, Point, 1, n_fingers> points = getFingerPoints(position);

            return Sphere<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2), points.getCoefficient(0, 3));
        }

        // Implemented here instead of in 'Hand.hxx' to avoid an issue in older compilers with 'requires'
        MultivectorMatrix<T, Sphere, 1, dof> getFingerSphereJacobian(const Eigen::Vector<T, dof> &position) const
            requires(n_fingers == 4)
        {
            auto finger_motors = getFingerMotors(position).asVector();
            auto finger_points = getFingerPoints(position).asVector();
            auto finger_jacobian = getAnalyticJacobian(position);

            MultivectorMatrix<T, Sphere, 1, dof> sphere_jacobian;

            for (int k = 0; k < finger_dof[0]; ++k)
            {
                Point<T> j_0_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[0].reverse()).evaluate();
                Point<T> j_0_2 = (finger_motors[0] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                sphere_jacobian.setCoefficient(0, k, (j_0_1 + j_0_2) ^ finger_points[1] ^ finger_points[2] ^ finger_points[3]);
            }

            for (int k = finger_dof[0]; k < finger_dof[0] + finger_dof[1]; ++k)
            {
                Point<T> j_1_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[1].reverse()).evaluate();
                Point<T> j_1_2 = (finger_motors[1] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                sphere_jacobian.setCoefficient(0, k, finger_points[0] ^ (j_1_1 + j_1_2) ^ finger_points[2] ^ finger_points[3]);
            }

            for (int k = finger_dof[0] + finger_dof[1]; k < finger_dof[2]; ++k)
            {
                Point<T> j_2_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[2].reverse()).evaluate();
                Point<T> j_2_2 = (finger_motors[2] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                sphere_jacobian.setCoefficient(0, k, finger_points[0] ^ finger_points[1] ^ (j_2_1 + j_2_2) ^ finger_points[3]);
            }

            for (int k = finger_dof[0] + finger_dof[1] + finger_dof[2]; k < dof; ++k)
            {
                Point<T> j_3_1 = finger_jacobian.getCoefficient(0, k) * (Point<T>() * finger_motors[3].reverse()).evaluate();
                Point<T> j_3_2 = (finger_motors[3] * Point<T>()).evaluate() * finger_jacobian.getCoefficient(0, k).reverse();

                sphere_jacobian.setCoefficient(0, k, finger_points[0] ^ finger_points[1] ^ finger_points[2] ^ (j_3_1 + j_3_2));
            }

            return sphere_jacobian;
        }

        // MultivectorMatrix<T, SimilarityTransformation, 1, dof> getAnalyticSimilarityJacobian(const Eigen::Vector<T, dof> &position) const
        //     requires(n_fingers == 3);

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

            FingerMotorFunctor(const Hand *hand) : FingerFunctor(hand) {}

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

            FingerAnalyticJacobianFunctor(const Hand *hand) : FingerFunctor(hand) {}

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

            FingerGeometricJacobianFunctor(const Hand *hand) : FingerFunctor(hand) {}

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