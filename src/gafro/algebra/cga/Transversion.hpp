// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Versor.hpp>

namespace gafro
{

    template <class T>
    class Transversion : public Versor<Transversion<T>, T, blades::scalar, blades::e01, blades::e02, blades::e03>
    {
      public:
        using Base = Versor<Transversion<T>, T, blades::scalar, blades::e01, blades::e02, blades::e03>;

        using Parameters = typename Base::Parameters;

        using Generator = Multivector<T, blades::e01, blades::e02, blades::e03>;

        using Base::Base;

        Transversion();

        Transversion(const Generator &generator);

        virtual ~Transversion();

        Generator log() const;

      protected:
      private:
      public:
        static Transversion exp(const T &e01, const T &e02, const T &e03);

        static Transversion exp(const Eigen::Vector<T, 3> &generator);

        static Transversion exp(const Generator &generator);
    };

}  // namespace gafro

#include <gafro/algebra/cga/Transversion.hxx>