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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/Rotor.hpp>
#include <gafro/algebra/Translator.hpp>
#include <gafro/algebra/expressions/Expression.hpp>

namespace gafro
{

    template <class T>
    class Motor;

    template <class Object, class Versor>
    class SandwichProduct;

    template <class T>
    class Motor : public Multivector<T, blades::scalar, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e123i>
    {
      public:
        using Base = Multivector<T, blades::scalar, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e123i>;

        using Type = typename Base::Type;

        using Parameters = typename Base::Parameters;

        class Generator;
        class Logarithm;
        class Exponential;

        using Base::Base;

        Motor();

        Motor(const Motor &other);

        Motor(Motor &&other);

        Motor(const Parameters &parameters);

        Motor(const Generator &generator);

        Motor(const Translator<T> &translator, const Rotor<T> &rotor);

        Motor(const Rotor<T> &rotor, const Translator<T> &translator);

        Motor(const Rotor<T> &rotor);

        Motor(const Base &other);

        Motor(Base &&other);

        template <class E>
        Motor(const Expression<E, Motor> &expression);

        virtual ~Motor() = default;

        Rotor<T> getRotor() const;

        Logarithm log() const;

        Eigen::Matrix<T, 6, 8> logJacobian() const;

      protected:
      private:
      public:
        Motor &operator=(const Motor &other);

        Motor &operator=(Motor &&other);

        Motor &operator=(Base &&other);

        template <class Object>
        SandwichProduct<Object, Motor> apply(const Object &object);

        template <class M>
        Motor &operator*=(const M &other)
        {
            *this = Motor(Motor(*this) * other);

            return *this;
        }

        using Base::operator=;

      public:
        static Motor<T> Unit();

        static Exponential exp(const Generator &generator);

      public:
    };

}  // namespace gafro

namespace Eigen
{
    template <class T>
    struct NumTraits<gafro::Motor<T>> : NumTraits<T>  // permits to get the epsilon, dummy_precision, lowest, highest functions
    {
        typedef gafro::Motor<T> Real;
        typedef gafro::Motor<T> NonInteger;
        typedef gafro::Motor<T> Nested;

        enum
        {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };
    };
}  // namespace Eigen