// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Reflection.hpp>
#include <gafro/algebra/cga/Sphere.hpp>
#include <gafro/algebra/cga/Translator.hpp>

namespace gafro
{

    template <typename T>
    Sphere<T>::Sphere()
      : Base()
    {}

    template <typename T>
    Sphere<T>::Sphere(const Base &other)
      : Base(other)
    {}

    template <typename T>
    Sphere<T>::Sphere(const Point<T> &center, const T &radius)  //
      : Base((center + Ei<T>(-0.5 * radius * radius)).evaluate().dual())
    {}

    template <typename T>
    Sphere<T>::Sphere(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3, const Point<T> &p4)  //
      : Base(p1 ^ p2 ^ p3 ^ p4)
    {}

    template <typename T>
    T Sphere<T>::getRadius() const
    {
        return sqrt(abs(((*this) * Scalar<T>(-1.0 / (this->dual() | Ei<T>(1.0)).template get<blades::scalar>())).evaluate().squaredNorm()));
    }

    template <typename T>
    Point<T> Sphere<T>::getCenter() const
    {
        auto dual_sphere = this->dual().evaluate();

        const typename Point<T>::Base ei({ 0.0, 0.0, 0.0, 0.0, 1.0 });

        Point<T> center = Reflection<typename Point<T>::Base, typename Dual<Sphere>::Type>(ei, dual_sphere).evaluate();
        center          = (center * Scalar<T>(1.0 / center.template get<blades::e0>())).evaluate();

        return center;
    }

    template <class T>
    SimilarityTransformation<T> Sphere<T>::getSimilarityTransformation(const Sphere &target) const
    {
        Dilator<T> dilator(target.getRadius() / this->getRadius());

        Sphere<T> sphere = dilator.apply(*this);

        Translator<T> translator = Translator<T>::exp(((target.getCenter() - sphere.getCenter()) ^ Ei<double>::One()));

        return translator * Rotor<T>::Unit() * dilator;
    }

    template <class T>
    Sphere<T> Sphere<T>::Random()
    {
        return Sphere(Point<T>::Random(), Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

    template <class T>
    Sphere<T> Sphere<T>::Unit()
    {
        return Sphere(Point<T>(), TypeTraits<T>::Value(1.0));
    }

}  // namespace gafro