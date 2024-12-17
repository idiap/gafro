#pragma once

#include <gafro/probabilistic/Point.hpp>

namespace gafro::probabilistic
{

    template <class T>
    Point<T>::Point(const Eigen::Vector<T, 3> &mean, const Eigen::Matrix<T, 3, 3> &covariance)
    {
        gafro::Point<T> point(mean.x(), mean.y(), mean.z());
        // point.template set<gafro::blades::ei>(point.template get<gafro::blades::ei>() + covariance.trace());

        Eigen::Matrix<T, 5, 3> jacobian = point.getEmbeddingJacobian();

        this->setMean(point);
        this->setCovariance(jacobian * covariance * jacobian.transpose());
    }

    template <class T>
    Point<T>::~Point() = default;

}  // namespace gafro::probabilistic
