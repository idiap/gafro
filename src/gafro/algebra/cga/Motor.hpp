// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Rotor.hpp>
#include <gafro/algebra/cga/Translator.hpp>

namespace gafro
{

    template <class T>
    class Motor;

    template <class Object, class Versor>
    class SandwichProduct;

    template <class T>
    class Motor
      : public Versor<Motor<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i, blades::e123i>
    {
      public:
        using Base = Versor<Motor<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i, blades::e123i>;

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

        Motor(const Eigen::Vector<T, 3> &position, const Eigen::Quaternion<T> &orientation);

        virtual ~Motor() = default;

        // OPERATORS

        Motor &operator=(const Motor &other);

        Motor &operator=(Motor &&other);

        Motor &operator*=(const Motor &other);

        // MOTOR SPECIFIC FUNCTIONS

        Rotor<T> getRotor() const;

        Translator<T> getTranslator() const;

        Logarithm log() const;

        Eigen::Matrix<T, 4, 4> toTransformationMatrix() const;

        Eigen::Matrix<T, 6, 6> toAdjointMatrix() const;

        Eigen::Matrix<T, 6, 6> toDualAdjointMatrix() const;

      public:
        // STATIC FUNCTIONS

        static Motor<T> Unit();

        static Motor<T> Random();

        static Exponential exp(const Generator &generator);

        static Exponential exp(const T &e12, const T &e13, const T &e23, const T &e1i, const T &e2i, const T &e3i);

      public:
    };

}  // namespace gafro