#pragma once

#include <gafro/geometry/ConformalPointcloud.hpp>
//
#include <gafro/algebra/cga/Motor.hxx>
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

    template <class T>
    ConformalPointcloud<T> ConformalPointcloud<T>::transform(const Motor<T> &motor) const
    {
        ConformalPointcloud<T> transformed_pointcloud;

        for (const auto &point : this->getPoints())
        {
            transformed_pointcloud->addPoint(motor.apply(point));
        }

        return transformed_pointcloud;
    }

}  // namespace gafro::geometry