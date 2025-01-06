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

#include <gafro/algebra/cga/Circle.hpp>
#include <gafro/algebra/cga/Plane.hpp>
#include <gafro/algebra/cga/Point.hpp>

namespace gafro
{

    template <typename T>
    Circle<T>::Circle(const Base &other) : Base(other)
    {}

    template <typename T>
    Circle<T>::Circle(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3) : Circle(p1 ^ p2 ^ p3)
    {}

    template <typename T>
    Point<T> Circle<T>::getCenter() const
    {
        Point<T> center((*this) * Ei<T>(T(1.0)) * (*this));

        center = center * Scalar<T>(T(1.0) / center.template get<blades::e0>());

        return center;
    }

    template <typename T>
    T Circle<T>::getRadius() const
    {
        auto mv = ((*this) * (Scalar<T>(T(-1.0)) * ((*this).dual() | Ei<T>(T(1.0))).evaluate().inverse())).evaluate();

        return sqrt(abs((mv * mv).template get<blades::scalar>()));
    }

    template <typename T>
    Plane<T> Circle<T>::getPlane() const
    {
        return Plane<T>((*this) ^ Ei<T>(T(1.0)));
    }

    template <typename T>
    ConformalTransformation<T> Circle<T>::getConformalTransformation(const Circle &target) const
    {
        if (this->norm() < 1e-5 || target.norm() < 1e-5)
        {
            return ConformalTransformation<T>::Zero();
        }

        Circle<T> circle = this->normalized();
        Circle<T> target_normalized = target.normalized();

        auto k = (Scalar<T>(TypeTraits<T>::Value(2.0)) + (circle * target_normalized + target_normalized * circle)).evaluate();
        auto k4 = (k.template extract<blades::e0123>()    //
                   + k.template extract<blades::e123i>()  //
                   + k.template extract<blades::e012i>()  //
                   + k.template extract<blades::e013i>()  //
                   + k.template extract<blades::e023i>())
                    .evaluate();

        if (k4.norm() < 1e-5)
        {
            return (Scalar<T>(TypeTraits<T>::Value(1.0)) + target_normalized * circle).evaluate().normalized();
        }

        auto k0 = k.template extract<blades::scalar>();
        T lambda = -(k4 * k4).template get<blades::scalar>();
        T mu = ((k0 - k4) * k).template get<blades::scalar>();
        T beta = sqrt(TypeTraits<T>::Value(1.0) / (TypeTraits<T>::Value(2.0) * lambda) * (sqrt(mu) - k0.template get<blades::scalar>()));

        return (Scalar<T>(TypeTraits<T>::Value(1.0) / sqrt(mu)) *
                (Scalar<T>(TypeTraits<T>::Value(-1.0) / (TypeTraits<T>::Value(2.0) * beta)) + Scalar<T>(beta) * k4) *
                (Scalar<T>(TypeTraits<T>::Value(1.0)) + target_normalized * circle))
          .evaluate();
    }

    template <class T>
    Motor<T> Circle<T>::getMotor(const Circle &target) const
    {
        Vector<T> n1 = this->getPlane().getNormal().normalized();
        Vector<T> n2 = target.getPlane().getNormal().normalized();
        Rotor<T> rotor = Rotor<T>(Scalar<T>(TypeTraits<T>::Value(1.0)) + (n1 | n2) - (n1 ^ n2)).normalized();

        Circle circle = rotor.apply(*this);

        Translator<T> translator = Translator<T>::exp(target.getCenter().getEuclideanPoint() - circle.getCenter().getEuclideanPoint());

        return Motor<T>(translator, rotor);
    }

    template <class T>
    SimilarityTransformation<T> Circle<T>::getSimilarityTransformation(const Circle &target) const
    {
        Dilator<T> dilator(target.getRadius() / this->getRadius());

        Circle<T> circle = dilator.apply(*this);

        Motor<T> motor = circle.getMotor(target);

        return motor * dilator;
    }

    template <class T>
    Circle<T> Circle<T>::Random()
    {
        return Circle(Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

    template <class T>
    Circle<T> Circle<T>::Unit(const Motor<T> &motor, const T &radius)
    {
        Point<T> p1(radius * TypeTraits<T>::Value(cos(0.0)), radius * TypeTraits<T>::Value(sin(0.0)), TypeTraits<T>::Value(0.0));
        Point<T> p2(radius * TypeTraits<T>::Value(cos(1.0)), radius * TypeTraits<T>::Value(sin(1.0)), TypeTraits<T>::Value(0.0));
        Point<T> p3(radius * TypeTraits<T>::Value(cos(2.0)), radius * TypeTraits<T>::Value(sin(2.0)), TypeTraits<T>::Value(0.0));

        return motor.apply(Circle<T>(p1, p2, p3));
    }

}  // namespace gafro