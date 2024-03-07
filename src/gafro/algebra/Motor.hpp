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
    class Motor
      : public Versor<Motor<T>, T, blades::scalar, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e123i>
    {
      public:
        using Base = Versor<Motor<T>, T, blades::scalar, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i, blades::e123i>;

        using Type = typename Base::Type;

        using Parameters = typename Base::Parameters;

        class Generator;
        class Logarithm;
        class Exponential;

        // BASE FUNCTIONS

        using Base::Base;

        // CONSTRUCTORS

        Motor();

        Motor(const Motor &other);

        Motor(Motor &&other);

        Motor(const Generator &generator);

        Motor(const Translator<T> &translator);

        Motor(const Rotor<T> &rotor);

        Motor(const Translator<T> &translator, const Rotor<T> &rotor);

        Motor(const Rotor<T> &rotor, const Translator<T> &translator);

        virtual ~Motor() = default;

        // OPERATORS

        Motor &operator=(const Motor &other);

        Motor &operator=(Motor &&other);

        Motor &operator*=(const Motor &other);

        // MOTOR SPECIFIC FUNCTIONS

        Rotor<T> getRotor() const;

        Translator<T> getTranslator() const;

        Logarithm log() const;

        Eigen::Matrix<T, 6, 8> logJacobian() const;

        Eigen::Matrix<T, 4, 4> toTransformationMatrix() const;

        Eigen::Matrix<T, 6, 6> toAdjointMatrix() const;

        Eigen::Matrix<T, 6, 6> toDualAdjointMatrix() const;

      public:
        // STATIC FUNCTIONS

        static Motor<T> Unit();

        static Motor<T> Random();

        static Exponential exp(const Generator &generator);

      public:
    };

}  // namespace gafro