#pragma once

#include <gafro/geometry/ConformalPointcloud.hpp>
//
#include <gafro/algebra/cga/Point.hxx>

namespace gafro::geometry
{

    template <class T>
    ConformalPointcloud<T>::ConformalPointcloud() = default;

    template <class T>
    ConformalPointcloud<T>::ConformalPointcloud(const EuclideanPointcloud<T> &pointcloud)
    {
        for (const auto &point : pointcloud.getPoints())
        {
            this->addPoint(Point<T>(point));
        }
    }

    template <class T>
    ConformalPointcloud<T>::~ConformalPointcloud() = default;

    template <class T>
    Eigen::VectorXd ConformalPointcloud<T>::getPointAsVector(const int &i) const
    {
        return this->getPoint(i).vector().template cast<double>();
    }

}  // namespace gafro::geometry