#pragma once

#include <gafro/geometry/Orbit.hpp>

namespace gafro::geometry
{

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    Orbit<T, blades...>::Orbit() = default;

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    Orbit<T, blades...>::Orbit(const Generator &generator) : generator_(generator)
    {}

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    Orbit<T, blades...>::~Orbit() = default;

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    void Orbit<T, blades...>::setGenerator(const Generator &generator)
    {
        generator_ = generator;
    }

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    const typename Orbit<T, blades...>::Generator &Orbit<T, blades...>::getGenerator() const
    {
        return generator_;
    }

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    EuclideanPointcloud<T> Orbit<T, blades...>::createEuclideanPointcloud(const Point<T> &point) const
    {
        EuclideanPointcloud<T> pointcloud;

        Eigen::Matrix<T, 3, 1> euclidean_point = point.getEuclideanPoint();

        for (int t = 0; t < 10000000; ++t)
        {
            Point<T> conformal_point(euclidean_point);

            Eigen::Matrix<T, 5, 3> jacobian = conformal_point.getEmbeddingJacobian();
            Eigen::Matrix<T, 3, 5> inverse_jacobian = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();

            euclidean_point += 1e-4 * inverse_jacobian * (Point<T>::Zero() + (conformal_point | generator_)).vector();

            pointcloud.addPoint(euclidean_point);
        }

        return pointcloud;
    }

}  // namespace gafro::geometry