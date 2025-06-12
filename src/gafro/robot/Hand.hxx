// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Hand.hpp>

namespace gafro
{

    template <class T, int... fingers>
    Hand<T, fingers...>::Hand(Hand &&hand)
    {
        *this = std::move(hand);
    }

    template <class T, int... fingers>
    Hand<T, fingers...> &Hand<T, fingers...>::operator=(Hand &&hand)
    {
        this->System<T>::operator=(std::move(hand));
        finger_tip_names_ = std::move(hand.finger_tip_names_);
    }

    template <class T, int... fingers>
    Hand<T, fingers...>::Hand(System<T> &&system, const std::array<std::string, n_fingers> &finger_tip_names)
      : System<T>(std::move(system)), finger_tip_names_(finger_tip_names)
    {
        for (int j = 0; j < n_fingers; ++j)
        {
            this->createKinematicChain(finger_tip_names[j]);
        }
    }

    template <class T, int... fingers>
    Hand<T, fingers...>::~Hand() = default;

    //

    template <class T, int... fingers>
    System<T> &Hand<T, fingers...>::getSystem()
    {
        return *this;
    }

    template <class T, int... fingers>
    const System<T> &Hand<T, fingers...>::getSystem() const
    {
        return *this;
    }

    //

    template <class T, int... fingers>
    template <int id>
    Motor<T> Hand<T, fingers...>::getFingerMotor(const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainMotor(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerAnalyticJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainAnalyticJacobian(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerGeometricJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position) const
    {
        assert(id < n_fingers);

        return this->computeKinematicChainGeometricJacobian(finger_tip_names_[id], position);
    }

    template <class T, int... fingers>
    template <int id>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::finger_dof[id]> Hand<T, fingers...>::getFingerGeometricJacobian(
      const Eigen::Vector<T, finger_dof[id]> &position, const Motor<T> &motor) const
    {
        return getFingerGeometricJacobian<id>(position).transform(motor.reverse());
    }

    //

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::n_fingers> Hand<T, fingers...>::getFingerMotors(const Eigen::Vector<T, dof> &position) const
    {
        FingerMotorFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.motors;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Point, 1, Hand<T, fingers...>::n_fingers> Hand<T, fingers...>::getFingerPoints(const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers> motors = getFingerMotors(position);

        MultivectorMatrix<T, Point, 1, n_fingers> points;

        for (int j = 0; j < n_fingers; ++j)
        {
            points.setCoefficient(0, j, motors.getCoefficient(0, j).apply(Point<T>()));
        }

        return points;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getAnalyticJacobian(const Eigen::Vector<T, dof> &position) const
    {
        FingerAnalyticJacobianFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getGeometricJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        FingerGeometricJacobianFunctor functor(this);

        forEachFinger(functor, position, std::make_index_sequence<n_fingers>());

        return functor.jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getGeometricJacobian(const Eigen::Vector<T, dof> &position,
                                                                                                                const Motor<T> &motor) const
    {
        return getGeometricJacobian(position).transform(motor.reverse());
    }

    template <class T, int... fingers>
    Motor<T> Hand<T, fingers...>::getMeanMotor(const Eigen::Vector<T, dof> &position) const
    {
        typename Motor<T>::Generator mean_bivector;

        for (const auto &m : getFingerMotors(position).asVector())
        {
            mean_bivector += m.log().evaluate();
        }

        return Motor<T>::exp(mean_bivector / static_cast<T>(n_fingers));
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Motor, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getMeanMotorAnalyticJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, Motor, 1, n_fingers> motors = getFingerMotors(position);

        Motor<T> mean_motor = getMeanMotor(position);

        Eigen::Matrix<T, 8, 6> exp_jacobian = Motor<T>::Exponential::getJacobian(mean_motor.log());

        MultivectorMatrix<T, Motor, 1, dof> jacobian;

        MultivectorMatrix<T, Motor, 1, dof> finger_jacobian = getAnalyticJacobian(position);

        for (int j = 0; j < n_fingers; ++j)
        {
            Eigen::Matrix<T, 6, 8> log_jacobian = Motor<T>::Logarithm::getJacobian(motors.getCoefficient(0, j));
            for (int k = 0; k < finger_dof[j]; ++k)
            {
                typename Motor<T>::Parameters parameters =
                  exp_jacobian * log_jacobian / static_cast<T>(n_fingers) * finger_jacobian.getCoefficient(0, k).vector();

                jacobian.setCoefficient(0, std::accumulate(finger_dof.begin(), finger_dof.begin() + j, 0) + k, Motor<T>(parameters));
            }
        }

        return jacobian;
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, MotorGenerator, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getMeanMotorGeometricJacobian(
      const Eigen::Vector<T, dof> &position) const
    {
        MultivectorMatrix<T, MotorGenerator, 1, dof> jacobian;

        Motor<T> mean_motor = getMeanMotor(position);

        MultivectorMatrix<T, Motor, 1, dof> analytic_jacobian = getMeanMotorAnalyticJacobian(position);

        for (int j = 0; j < dof; ++j)
        {
            jacobian.setCoefficient(0, j, -2.0 * analytic_jacobian.getCoefficient(0, j) * mean_motor.reverse());
        }

        return jacobian;
    }

    template <class T, int... fingers>
    Circle<T> Hand<T, fingers...>::getFingerCircle(const Eigen::Vector<T, dof> &position) const
        requires(n_fingers == 3)
    {
        MultivectorMatrix<T, Point, 1, n_fingers> points = getFingerPoints(position);

        return Circle<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2));
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Circle, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getFingerCircleJacobian(
      const Eigen::Vector<T, dof> &position) const
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

    template <class T, int... fingers>
    Sphere<T> Hand<T, fingers...>::getFingerSphere(const Eigen::Vector<T, dof> &position) const
        requires(n_fingers == 4)
    {
        MultivectorMatrix<T, Point, 1, n_fingers> points = getFingerPoints(position);

        return Sphere<T>(points.getCoefficient(0, 0), points.getCoefficient(0, 1), points.getCoefficient(0, 2), points.getCoefficient(0, 3));
    }

    template <class T, int... fingers>
    MultivectorMatrix<T, Sphere, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getFingerSphereJacobian(
      const Eigen::Vector<T, dof> &position) const
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

    // template <class T, int... fingers>
    // MultivectorMatrix<T, SimilarityTransformation, 1, Hand<T, fingers...>::dof> Hand<T, fingers...>::getAnalyticSimilarityJacobian(
    //   const Eigen::Vector<T, dof> &position) const
    //     requires(n_fingers == 3)
    // {
    //     Circle<T> circle = getFingerCircle(position);
    //     auto circle_jacobian = getFingerCircleJacobian(position);

    //     auto sim = Circle<T>::Unit(gafro::Motor<T>()).getSimilarityTransformation(circle);
    //     auto translator = sim.getCanonicalDecomposition().getTranslator();
    //     auto rotor = sim.getCanonicalDecomposition().getRotor();
    //     auto dilator = sim.getCanonicalDecomposition().getDilator();

    //     Point<T> p = circle * Ei<T>::One() * circle;
    //     auto center_jacobian =
    //       (circle_jacobian * (Ei<T>::One() * circle).evaluate() + (circle * Ei<T>::One()).evaluate() * circle_jacobian).template extract<Point>();

    //     auto translator_jacobian =
    //       -0.5 * (std::pow(p.template get<blades::e0>(), -1.0) * center_jacobian +
    //               -std::pow(p.template get<blades::e0>(), -2.0) * p * (Ei<T>(-1.0) | center_jacobian.template extract<E0>())) ^
    //       Ei<T>::One();

    //     Vector<T> unit_normal = Circle<T>(dilator.apply(Circle<T>::Unit(gafro::Motor<T>()))).getPlane().getNormal().normalized();
    //     Vector<T> circle_normal_unnormalized = circle.getPlane().getNormal();
    //     Vector<T> circle_normal = circle_normal_unnormalized.normalized();
    //     Rotor<T> unnormalized_rotor = Scalar<T>::One() + (unit_normal | circle_normal) - (unit_normal ^ circle_normal);

    //     auto circle_plane_jacobian = circle_jacobian ^ Ei<T>::One();
    //     auto circle_normal_jacobian_unnormalized =
    //       (circle_plane_jacobian.dual() + -0.5 * (E0<T>::One() | circle_plane_jacobian.dual()) * Ei<T>::One()).template extract<Vector>();

    //     auto circle_normal_jacobian =
    //       (1.0 / circle_normal_unnormalized.norm()) * circle_normal_jacobian_unnormalized -
    //       0.5 * std::pow(circle_normal_unnormalized.norm(), -3.0) *
    //         (circle_normal_unnormalized * (circle_normal_unnormalized * circle_normal_jacobian_unnormalized.reverse() +
    //                                        circle_normal_jacobian_unnormalized * circle_normal_unnormalized.reverse().evaluate()))
    //           .template extract<Vector>();

    //     auto rotor_jacobian_unnormalized = (unit_normal | circle_normal_jacobian) - (unit_normal ^ circle_normal_jacobian);

    //     auto rotor_jacobian = (1.0 / unnormalized_rotor.norm()) * rotor_jacobian_unnormalized -
    //                           0.5 * std::pow(unnormalized_rotor.norm(), -3.0) *
    //                             (unnormalized_rotor * (rotor_jacobian_unnormalized * unnormalized_rotor.reverse().evaluate() +
    //                                                    unnormalized_rotor * rotor_jacobian_unnormalized.reverse()));

    //     MultivectorMatrix<T, Circle, 1, 7> circle_jacobian_reversed = circle_jacobian.reverse();
    //     auto circle_jacobian_inversed = (1.0 / circle.squaredNorm()) * circle_jacobian.reverse()  //
    //                                     + -std::pow(circle.squaredNorm(), -2.0) *
    //                                         (circle.reverse().evaluate() * (2.0 * (circle * circle_jacobian_reversed)).template extract<Scalar>());

    //     auto e = ((Ei<T>::One() | circle) * circle.inverse()).evaluate();
    //     auto j = ((Ei<T>(1.0) | circle_jacobian) * circle.inverse().evaluate() + (Ei<T>(1.0) | circle).evaluate() * circle_jacobian_inversed);

    //     auto dilator_jacobian = dilator * -0.25 * std::pow(abs(e.squaredNorm()), -1.0)  //
    //                             * (j * e.reverse().evaluate() + e * j.reverse()).template extract<Scalar>();

    //     return translator_jacobian * (rotor * dilator).evaluate() + translator * rotor_jacobian * dilator +
    //            (translator * rotor).evaluate() * dilator_jacobian;
    // }

}  // namespace gafro