// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/cga/Blades.hpp>

namespace gafro
{
    template <class T>
    class Point : public Multivector<T, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>
    {
      public:
        using Base = Multivector<T, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Point();

        Point(const Base &other);

        Point(const Eigen::Vector3<T> &point);

        Point(const T &x, const T &y, const T &z);

        virtual ~Point() = default;

        Eigen::Matrix<T, 5, 3> getEmbeddingJacobian() const;

        Eigen::Matrix<T, 3, 1> getEuclideanPoint() const;

      protected:
      private:
      public:
        static Point X(const T &x = T(1.0));

        static Point Y(const T &y = T(1.0));

        static Point Z(const T &z = T(1.0));

        static Point Random();
    };

}  // namespace gafro