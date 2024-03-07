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

#include <gafro/algebra/Multivector.hpp>
#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{

    namespace blades
    {
        constexpr short scalar = 0;
        constexpr short e1 = 1;
        constexpr short e2 = 2;
        constexpr short e3 = 3;
        constexpr short ei = 4;
        constexpr short e0 = 5;
        //
        constexpr short e23 = 6;
        constexpr short e13 = 7;
        constexpr short e12 = 8;
        constexpr short e1i = 9;
        constexpr short e2i = 10;
        constexpr short e3i = 11;
        constexpr short e01 = 12;
        constexpr short e02 = 13;
        constexpr short e03 = 14;
        constexpr short e0i = 15;
        //
        constexpr short e123 = 16;
        constexpr short e12i = 17;
        constexpr short e13i = 18;
        constexpr short e23i = 19;
        constexpr short e012 = 20;
        constexpr short e013 = 21;
        constexpr short e023 = 22;
        constexpr short e01i = 23;
        constexpr short e02i = 24;
        constexpr short e03i = 25;
        //
        constexpr short e123i = 26;
        constexpr short e0123 = 27;
        constexpr short e012i = 28;
        constexpr short e023i = 29;
        constexpr short e013i = 30;
        //
        constexpr short e0123i = 31;
    }   // namespace blades

    template <class T, int index>
    class Blade : public Multivector<T, index>
    {
      public:
        using Base = Multivector<T, index>;

        using Base::Base;

        Blade(const Base &other) : Base(other) {}

        Blade(const T &value) : Base((Eigen::Matrix<T, 1, 1>() << value).finished()) {}

        virtual ~Blade() = default;

        using Multivector<T, index>::blades;

        const T &value() const
        {
            return this->template get<index>();
        }

      public:
      private:
    };

    template <class T>
    class Scalar : public Blade<T, blades::scalar>
    {
      public:
        constexpr static int index = blades::scalar;

      public:
        using Blade<T, index>::Blade;

        using Base = typename Blade<T, index>::Base;

        Scalar() : Blade<T, index>(T(0.0)) {}

        Scalar(const T &value) : Blade<T, index>(value) {}

        template <class E>
        Scalar(const Expression<E, Scalar> &expression) : Scalar(expression.template get<blades::scalar>())
        {}

        virtual ~Scalar() = default;

        using Blade<T, index>::blades;
        constexpr static int size = 1;

      public:
      private:
    };

    template <class T>
    class E1 : public Blade<T, blades::e1>
    {
      public:
        constexpr static int index = blades::e1;

      public:
        using Blade<T, index>::Blade;

        virtual ~E1() = default;

      private:
    };

    template <class T>
    class E2 : public Blade<T, blades::e2>
    {
      public:
        constexpr static int index = blades::e2;

      public:
        using Blade<T, index>::Blade;

        virtual ~E2() = default;

        using Blade<T, index>::blades;
        constexpr static int size = 1;

      private:
    };

    template <class T>
    class E3 : public Blade<T, blades::e3>
    {
      public:
        constexpr static int index = blades::e3;

      public:
        using Blade<T, index>::Blade;

        virtual ~E3() = default;

      private:
    };

    template <class T>
    class Ei : public Blade<T, blades::ei>
    {
      public:
        constexpr static int index = blades::ei;

      public:
        using Blade<T, index>::Blade;

        virtual ~Ei() = default;

      private:
    };

    template <class T>
    class E0 : public Blade<T, blades::e0>
    {
      public:
        constexpr static int index = blades::e0;

      public:
        using Blade<T, index>::Blade;

        virtual ~E0() = default;

      private:
    };

    template <class T>
    class E23 : public Blade<T, blades::e23>
    {
      public:
        constexpr static int index = blades::e23;

      public:
        using Blade<T, index>::Blade;

        virtual ~E23() = default;

      private:
    };

    template <class T>
    class E13 : public Blade<T, blades::e13>
    {
      public:
        constexpr static int index = blades::e13;

      public:
        using Blade<T, index>::Blade;

        virtual ~E13() = default;

      private:
    };

    template <class T>
    class E12 : public Blade<T, blades::e12>
    {
      public:
        constexpr static int index = blades::e12;

      public:
        using Blade<T, index>::Blade;

        virtual ~E12() = default;

      private:
    };

    template <class T>
    class E1i : public Blade<T, blades::e1i>
    {
      public:
        constexpr static int index = blades::e1i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E1i() = default;

      private:
    };

    template <class T>
    class E2i : public Blade<T, blades::e2i>
    {
      public:
        constexpr static int index = blades::e2i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E2i() = default;

      private:
    };

    template <class T>
    class E3i : public Blade<T, blades::e3i>
    {
      public:
        constexpr static int index = blades::e3i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E3i() = default;

      private:
    };

    template <class T>
    class E01 : public Blade<T, blades::e01>
    {
      public:
        constexpr static int index = blades::e01;

      public:
        using Blade<T, index>::Blade;

        virtual ~E01() = default;

      private:
    };

    template <class T>
    class E02 : public Blade<T, blades::e02>
    {
      public:
        constexpr static int index = blades::e02;

      public:
        using Blade<T, index>::Blade;

        virtual ~E02() = default;

      private:
    };

    template <class T>
    class E03 : public Blade<T, blades::e03>
    {
      public:
        constexpr static int index = blades::e03;

      public:
        using Blade<T, index>::Blade;

        virtual ~E03() = default;

      private:
    };

    template <class T>
    class E0i : public Blade<T, blades::e0i>
    {
      public:
        constexpr static int index = blades::e0i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E0i() = default;

      private:
    };

    template <class T>
    class E123 : public Blade<T, blades::e123>
    {
      public:
        constexpr static int index = blades::e123;

      public:
        using Blade<T, index>::Blade;

        virtual ~E123() = default;

      private:
    };

    template <class T>
    class E12i : public Blade<T, blades::e12i>
    {
      public:
        constexpr static int index = blades::e12i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E12i() = default;

      private:
    };

    template <class T>
    class E13i : public Blade<T, blades::e13i>
    {
      public:
        constexpr static int index = blades::e13i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E13i() = default;

      private:
    };

    template <class T>
    class E23i : public Blade<T, blades::e23i>
    {
      public:
        constexpr static int index = blades::e23i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E23i() = default;

      private:
    };

    template <class T>
    class E012 : public Blade<T, blades::e012>
    {
      public:
        constexpr static int index = blades::e012;

      public:
        using Blade<T, index>::Blade;

        virtual ~E012() = default;

      private:
    };

    template <class T>
    class E013 : public Blade<T, blades::e013>
    {
      public:
        constexpr static int index = blades::e013;

      public:
        using Blade<T, index>::Blade;

        virtual ~E013() = default;

      private:
    };

    template <class T>
    class E023 : public Blade<T, blades::e023>
    {
      public:
        constexpr static int index = blades::e023;

      public:
        using Blade<T, index>::Blade;

        virtual ~E023() = default;

      private:
    };

    template <class T>
    class E01i : public Blade<T, blades::e01i>
    {
      public:
        constexpr static int index = blades::e01i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E01i() = default;

      private:
    };

    template <class T>
    class E02i : public Blade<T, blades::e02i>
    {
      public:
        constexpr static int index = blades::e02i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E02i() = default;

      private:
    };

    template <class T>
    class E03i : public Blade<T, blades::e03i>
    {
      public:
        constexpr static int index = blades::e03i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E03i() = default;

      private:
    };

    template <class T>
    class E123i : public Blade<T, blades::e123i>
    {
      public:
        constexpr static int index = blades::e123i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E123i() = default;

      private:
    };

    template <class T>
    class E0123 : public Blade<T, blades::e0123>
    {
      public:
        constexpr static int index = blades::e0123;

      public:
        using Blade<T, index>::Blade;

        virtual ~E0123() = default;

      private:
    };

    template <class T>
    class E012i : public Blade<T, blades::e012i>
    {
      public:
        constexpr static int index = blades::e012i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E012i() = default;

      private:
    };

    template <class T>
    class E023i : public Blade<T, blades::e023i>
    {
      public:
        constexpr static int index = blades::e023i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E023i() = default;

      private:
    };

    template <class T>
    class E013i : public Blade<T, blades::e013i>
    {
      public:
        constexpr static int index = blades::e013i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E013i() = default;

      private:
    };

    template <class T>
    class E0123i : public Blade<T, blades::e0123i>
    {
      public:
        constexpr static int index = blades::e0123i;

      public:
        using Blade<T, index>::Blade;

        virtual ~E0123i() = default;

      private:
    };

}  // namespace gafro
