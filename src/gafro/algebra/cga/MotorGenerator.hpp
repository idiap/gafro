// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Expression.hpp>
#include <gafro/algebra/cga/Motor.hpp>
#include <gafro/algebra/cga/RotorGenerator.hpp>

namespace gafro
{

    template <class T>
    class Motor<T>::Generator : public Multivector<T, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i>
    {
      public:
        using Base = Multivector<T, blades::e12, blades::e13, blades::e23, blades::e1i, blades::e2i, blades::e3i>;

        using Parameters = typename Base::Parameters;

        //

        using Base::Base;
        using Base::operator=;

        //

        Generator();

        Generator(const Base &other);

        Generator(const Parameters &parameters);

        Generator(const Eigen::Matrix<T, 3, 1> &p1, const Eigen::Matrix<T, 3, 1> &p2);

        virtual ~Generator() = default;

        typename Rotor<T>::Generator getRotorGenerator() const;

        typename Translator<T>::Generator getTranslatorGenerator() const;

      protected:
      private:
    };

    template <class T>
    using MotorGenerator = typename Motor<T>::Generator;

}  // namespace gafro