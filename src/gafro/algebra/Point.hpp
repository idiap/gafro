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

namespace gafro
{
    template <class T>
    class Point : public Multivector<T, blades::e1, blades::e2, blades::e3, blades::ei, blades::e0>
    {
      public:
        using Base = Multivector<T, blades::e1, blades::e2, blades::e3, blades::ei, blades::e0>;

        using Parameters = typename Base::Parameters;

        using Base::Base;

        Point();

        Point(const Base &other);

        Point(const T &x, const T &y, const T &z);

        virtual ~Point() = default;

        template <class E>
        // requires(std::is_same<Point, typename Expression::Type>::value || std::is_convertible<Base, typename Expression::Type>::value)  //
        Point(const Expression<E, Point> &expression)
        {
            this->template set<blades::e0>(expression.template get<blades::e0>());
            this->template set<blades::e1>(expression.template get<blades::e1>());
            this->template set<blades::e2>(expression.template get<blades::e2>());
            this->template set<blades::e3>(expression.template get<blades::e3>());
            this->template set<blades::ei>(expression.template get<blades::ei>());
        }

        // template <class T2>
        // Point(const Point<T2> &point);

        // Point &operator=(const Point &point);

        // T distance(const Point &point) const;

        // Point toSphere(const Sphere<T> &sphere) const;

        // Point toSphere(const Point &center, const T &radius) const;

        // Eigen::Matrix<T, 3, 1> toEuclidean() const;

        // Vector<T> toVector() const;

        // PointPair<T> operator^(const Point &point) const;

        // Point operator*(const Point &point) const;

        // Point operator*(const T &val);

        // Point &operator+=(const Point &point);

        // T operator|(const Point &other) const;

        // Point operator-(const Point &other) const;

        // using Multivector<T>::norm;

        // using Multivector<T>::mv;

        // friend std::ostream &operator<<(std::ostream &ostream, const Point<T> &point)
        // {
        //     ostream << point.mv();
        //     return ostream;
        // }

      protected:
      private:
      public:
    };

}  // namespace gafro

namespace Eigen
{
    template <class T>
    struct NumTraits<gafro::Point<T>> : NumTraits<T>  // permits to get the epsilon, dummy_precision, lowest, highest functions
    {
        typedef gafro::Point<T> Real;
        typedef gafro::Point<T> NonInteger;
        typedef gafro::Point<T> Nested;

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