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

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Twist;

    template <class T>
    class Wrench;

    template <class T>
    using InertiaElement = Multivector<T, blades::e23, blades::e13, blades::e12, blades::e01, blades::e02, blades::e03>;

    template <class T>
    class Inertia : private MultivectorMatrix<T, InertiaElement, 1, 6>
    {
      public:
        using Base = MultivectorMatrix<T, InertiaElement, 1, 6>;

        Inertia();

        Inertia(const T &mass, const T &ixx, const T &ixy, const T &ixz, const T &iyy, const T &iyz, const T &izz);

        Inertia(const T &mass, const Eigen::Matrix<T, 3, 3> &tensor);

        Inertia(const std::array<InertiaElement<T>, 6> &elements);

        template <class S>
        Inertia(const Inertia<S> &other);

        virtual ~Inertia() = default;

        //

        Inertia &operator+=(const Inertia &inertia);

        Inertia operator+(const Inertia &inertia);

        //

        Wrench<T> operator()(const Twist<T> &twist) const;

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

      protected:
      private:
      public:
        static Inertia Zero();

        // friend std::ostream &operator<<(std::ostream &ostream, const Inertia &inertia);
    };

}  // namespace gafro