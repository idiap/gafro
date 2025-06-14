// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Eigenvalues>
#include <gafro/algebra/cga/Plane.hpp>
#include <gafro/algebra/cga/Point.hpp>
#include <gafro/algebra/cga/Vector.hpp>

namespace gafro
{
    template <typename T>
    Plane<T>::Plane(const Base &other) : Base(other)
    {}

    template <typename T>
    Plane<T>::Plane(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3)  //
      : Base(p1 ^ p2 ^ p3 ^ Ei<T>(T(1.0)))
    {}

    template <typename T>
    Vector<T> Plane<T>::getNormal() const
    {
        return this->dual() - Scalar<T>(T(0.5)) * (this->dual() | E0<T>(T(1.0))) * Ei<T>(T(1.0));
    }

    template <typename T>
    Motor<T> Plane<T>::getMotor(const Plane &other) const
    {
        Plane<T> plane1 = this->normalized();
        Plane<T> plane2 = other.normalized();

        auto motor_exp = ((Scalar<double>::One() - plane2 * plane1) *
                          Scalar<double>(1.0 / std::sqrt((Scalar<double>(2.0) - (plane1 * plane2 + plane2 * plane1)).template get<blades::scalar>())))
                           .evaluate();

        Motor<T> motor = motor_exp + E123i<double>::Zero();

        return motor;
    }

    template <typename T>
    Plane<T> Plane<T>::XY(const T &z)
    {
        return Plane(Point<T>(0.0, 0.0, z), Point<T>(1.0, 0.0, z), Point<T>(0.0, 1.0, z));
    }

    template <typename T>
    Plane<T> Plane<T>::XZ(const T &y)
    {
        return Plane(Point<T>(0.0, y, 0.0), Point<T>(1.0, y, 0.0), Point<T>(0.0, y, 1.0));
    }

    template <typename T>
    Plane<T> Plane<T>::YZ(const T &x)
    {
        return Plane(Point<T>(x, 0.0, 0.0), Point<T>(x, 1.0, 0.0), Point<T>(x, 0.0, 1.0));
    }

    template <class T>
    Plane<T> Plane<T>::Random()
    {
        return Plane(Point<T>::Random(), Point<T>::Random(), Point<T>::Random());
    }

    template <class T>
    Plane<T> Plane<T>::estimateFromPoints(const std::vector<Point<T>> &points, const Eigen::VectorXi &indices)
    {
        auto weight = [&](unsigned k, const Point<double> &p) {
            switch (k)
            {
            case 0:
                return p.template get<blades::e1>();
            case 1:
                return p.template get<blades::e2>();
            case 2:
                return p.template get<blades::e3>();
            case 3:
                return -p.template get<blades::ei>();
            default:
                return 0.0;
            }
        };

        Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(4, 4);

        for (unsigned i = 0; i < indices.rows(); ++i)
        {
            const auto &point = points[indices[i]];

            for (unsigned j = 0; j < 4; ++j)
            {
                for (unsigned k = 0; k < 4; ++k)
                {
                    weights.coeffRef(j, k) += weight(j, point) * weight(k, point);
                }
            }
        }

        Eigen::EigenSolver<Eigen::MatrixXd> solver(weights);

        Eigen::Index min;
        solver.eigenvalues().unaryExpr([](auto &v) { return v.real(); }).minCoeff(&min);

        Multivector<double, blades::e1, blades::e2, blades::e3, blades::ei> dual_plane;
        dual_plane.set<blades::e1>(solver.eigenvectors().coeffRef(0, min).real());
        dual_plane.set<blades::e2>(solver.eigenvectors().coeffRef(1, min).real());
        dual_plane.set<blades::e3>(solver.eigenvectors().coeffRef(2, min).real());
        dual_plane.set<blades::ei>(solver.eigenvectors().coeffRef(3, min).real());

        return dual_plane.dual();
    }

    template <class T>
    Plane<T> Plane<T>::estimateFromPoints(const std::vector<Point<T>> &points)
    {
        std::vector<unsigned> indices;
        std::iota(indices.begin(), indices.end(), 0);

        return estimateFromPoints(points, indices);
    }

}  // namespace gafro