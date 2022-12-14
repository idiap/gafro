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

#include <gafro/algebra/Point.hpp>

namespace gafro
{

    template <class T>
    Point<T>::Point() : Point<T>(0.0, 0.0, 0.0)
    {}

    template <class T>
    Point<T>::Point(const T &x, const T &y, const T &z)
    {
        Parameters parameters = Parameters::Ones();

        parameters.coeffRef(0, 0) = x;
        parameters.coeffRef(1, 0) = y;
        parameters.coeffRef(2, 0) = z;
        parameters.coeffRef(3, 0) = 0.5 * parameters.topRows(3).squaredNorm();
        parameters.coeffRef(4, 0) = 1.0;

        this->setParameters(parameters);
    }

    template <class T>
    Point<T>::Point(const Base &other) : Base(other)
    {}

    // template <class T>
    // Point<T> &Point<T>::operator=(const Point &point)
    // {
    //     mv() = point.mv();

    //     return *this;
    // }

    // template <class T>
    // T Point<T>::distance(const Point &other) const
    // {
    //     return sqrt(abs(T(-2.0) * Multivector<T>::scalarProduct(mv(), other.mv())));
    //     // return (*this - other).mv().norm();
    // }

    // template <class T>
    // Point<T> Point<T>::toSphere(const Sphere<T> &sphere) const
    // {
    //     Multivector<T> dual_sphere = sphere.mv().dual();

    //     Multivector<T> dual_line = (mv() ^ dual_sphere ^ blades::Ei<T>()).dual();

    //     return Point<T>(PointPair<T>(dual_line, dual_sphere).getPoint1());
    // }

    // template <class T>
    // Point<T> Point<T>::toSphere(const Point<T> &center, const T &radius) const
    // {
    //     Multivector<T> dual_sphere = Sphere<T>(center, radius);

    //     Multivector<T> line = mv() ^ center ^ blades::Ei<T>();

    //     Multivector<T> pp = line | dual_sphere;

    //     // std::cout << "-----" << std::endl;
    //     // std::cout << pp << std::endl;
    //     // std::cout << pp.norm() << std::endl;
    //     // std::cout << pp + pp.norm() << std::endl;
    //     // std::cout << pp - pp.norm() << std::endl;
    //     // std::cout << (pp + pp.norm()) * (blades::Ei(-1.0) | pp).inverse() << std::endl;
    //     // std::cout << (pp - pp.norm()) * (blades::Ei(-1.0) | pp).inverse() << std::endl;
    //     // std::cout << (1 + pp / std::sqrt(std::abs((pp * pp.reverse()).scalar()))) * (pp | blades::Ei()) << std::endl;

    //     return PointPair<T>(pp).getPoint2();
    // }

    // template <class T>
    // Eigen::Matrix<T, 3, 1> Point<T>::toEuclidean() const
    // {
    //     Eigen::Matrix<T, 3, 1> vector;

    //     vector.x() = mv().select(blades::E1<T>());
    //     vector.y() = mv().select(blades::E2<T>());
    //     vector.z() = mv().select(blades::E3<T>());

    //     vector /= mv().select(blades::E0<T>());

    //     return vector;
    // }

    // template <class T>
    // Vector<T> Point<T>::toVector() const
    // {
    //     return Vector(toEuclidean());
    // }

    // template <class T>
    // PointPair<T> Point<T>::operator^(const Point &point) const
    // {
    //     return mv() ^ point.mv();
    // }

    // template <class T>
    // Point<T> Point<T>::operator*(const Point &point) const
    // {
    //     return mv() * point.mv();
    // }

    // template <class T>
    // Point<T> Point<T>::operator*(const T &val)
    // {
    //     return val * mv();
    // }

    // template <class T>
    // Point<T> &Point<T>::operator+=(const Point &other)
    // {
    //     mv() += other.mv();

    //     return *this;
    // }

    // template <class T>
    // Point<T> operator*(const double &val, const Point<T> &point)
    // {
    //     return val * point.mv();
    // }

    // template <class T>
    // T Point<T>::operator|(const Point &other) const
    // {
    //     return (mv() | other.mv()).select(blades::Scalar<T>());
    // }

    // template <class T>
    // Point<T> Point<T>::operator-(const Point &other) const
    // {
    //     return mv() - other.mv();
    // }

}  // namespace gafro