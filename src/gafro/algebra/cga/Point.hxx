// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/cga/Point.hpp>

namespace gafro
{

    template <class T>
    Point<T>::Point() : Point<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    Point<T>::Point(const T &x, const T &y, const T &z)
    {
        this->template set<blades::e1>(x);
        this->template set<blades::e2>(y);
        this->template set<blades::e3>(z);
        this->template set<blades::ei>(T(0.5) * (x * x + y * y + z * z));
        this->template set<blades::e0>(T(1.0));
    }

    template <class T>
    Point<T>::Point(const Base &other) : Base(other)
    {}

    template <class T>
    Point<T>::Point(const Eigen::Vector3<T> &point) : Point(point.x(), point.y(), point.z())
    {}

    template <class T>
    Point<T> Point<T>::X(const T &x)
    {
        return Point(x, T(0.0), T(0.0));
    }

    template <class T>
    Point<T> Point<T>::Y(const T &y)
    {
        return Point(T(0.0), y, T(0.0));
    }

    template <class T>
    Point<T> Point<T>::Z(const T &z)
    {
        return Point(T(0.0), T(0.0), z);
    }

    template <class T>
    Point<T> Point<T>::Random()
    {
        Eigen::Vector3<T> p = Eigen::Vector3<T>::Random();

        return Point(p.x(), p.y(), p.z());
    }

    template <class T>
    Eigen::Matrix<T, 5, 3> Point<T>::getEmbeddingJacobian() const
    {
        Eigen::Matrix<T, 5, 3> jacobian = Eigen::Matrix<T, 5, 3>::Zero();

        jacobian.middleRows(1, 3) = Eigen::Matrix<T, 3, 3>::Identity();

        jacobian.bottomRows(1) = this->getEuclideanPoint().transpose();

        return jacobian;
    }

    template <class T>
    Eigen::Matrix<T, 3, 1> Point<T>::getEuclideanPoint() const
    {
        Eigen::Matrix<T, 3, 1> point;

        point[0] = this->template get<blades::e1>();
        point[1] = this->template get<blades::e2>();
        point[2] = this->template get<blades::e3>();

        return point;
    }

}  // namespace gafro