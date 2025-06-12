// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Geometry>
//
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{
    template <class Object, class Versor>
    class SandwichProduct;

    template <class T>
    class Rotor : public Versor<Rotor<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23>
    {
      public:
        using Base = Versor<Rotor<T>, T, blades::scalar, blades::e12, blades::e13, blades::e23>;

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
        static Exponential exp(const T &e12, const T &e13, const T &e23);

        static Exponential exp(const Eigen::Vector<T, 3> &generator);

        static Exponential exp(const Generator &generator);

        static Rotor fromQuaternion(const Eigen::Quaternion<T> &quaternion);

        static Rotor Unit();
    };

}  // namespace gafro