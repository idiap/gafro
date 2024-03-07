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

#include <Eigen/Geometry>
//
#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/Versor.hpp>

namespace gafro
{
    template <class Object, class Versor>
    class SandwichProduct;

    template <class T>
    class Rotor : public Versor<Rotor<T>, T, blades::scalar, blades::e23, blades::e13, blades::e12>
    {
      public:
        using Base = Versor<Rotor<T>, T, blades::scalar, blades::e23, blades::e13, blades::e12>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        class Generator;
        class Exponential;

        Rotor();

        Rotor(const Generator &bivector, const T &angle);

        Rotor(const Parameters &parameters);

        virtual ~Rotor();

        T angle() const;

        Generator log() const;

        Eigen::Quaternion<T> quaternion() const;

        Eigen::Matrix<T, 3, 3> toRotationMatrix() const;

        const T &scalar() const;

        const T &e23() const;

        const T &e13() const;

        const T &e12() const;

      protected:
      private:
      public:
        static Exponential exp(const Generator &generator);

        static Rotor fromQuaternion(const Eigen::Quaternion<T> &quaternion);
    };

}  // namespace gafro