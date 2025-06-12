// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Twist;

    template <class T>
    class Wrench;

    template <class T>
    using InertiaElement = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>;

    template <class T>
    class Inertia : private MultivectorMatrix<T, InertiaElement, 1, 6>
    {
      public:
        using Base = MultivectorMatrix<T, InertiaElement, 1, 6>;

        Inertia();

        Inertia(const T &mass, const T &ixx, const T &ixy, const T &ixz, const T &iyy, const T &iyz, const T &izz);

        Inertia(const T &mass, const Eigen::Matrix<T, 3, 3> &tensor);

        Inertia(const Eigen::Matrix<T, 3, 3> &translation, const Eigen::Matrix<T, 3, 3> &rotational);

        Inertia(const std::array<InertiaElement<T>, 6> &elements);

        template <class S>
        Inertia(const Inertia<S> &other);

        Inertia(const Base &base);

        virtual ~Inertia() = default;

        //

        Inertia &operator+=(const Inertia &inertia);

        Inertia operator+(const Inertia &inertia);

        //

        /**
         * @brief linear map from Twist to Wrench
         * @details
         *
         * @param twist input Twist
         * @return Wrench
         */
        Wrench<T> operator()(const Twist<T> &twist) const;

        /**
         * @brief inverse map from Wrench to Twist
         * @details
         *
         * @param wrench input Wrench
         * @return Twist
         */
        Twist<T> operator()(const Wrench<T> &wrench) const;

        //

        Inertia transform(const Motor<T> &motor) const;

        Inertia inverseTransform(const Motor<T> &motor) const;

        //

        const InertiaElement<T> &getElement23() const;

        const InertiaElement<T> &getElement13() const;

        const InertiaElement<T> &getElement12() const;

        const InertiaElement<T> &getElement01() const;

        const InertiaElement<T> &getElement02() const;

        const InertiaElement<T> &getElement03() const;

        //

        typename Base::Matrix getTensor() const;

        const Base &getMultivectorMatrix() const;

      protected:
      private:
      public:
        static Inertia Zero();

        static Inertia Random();

        static Inertia RandomDiagonal();
    };

}  // namespace gafro