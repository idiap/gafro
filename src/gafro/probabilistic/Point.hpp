#pragma once

#include <gafro/gafro.hpp>
#include <gafro/probabilistic/MultivectorGaussian.hpp>

namespace gafro::probabilistic
{

    template <class T = double>
    class Point : public gafro::Point<T>::Gaussian
    {
      public:
        Point(const Eigen::Vector<T, 3> &mean, const Eigen::Matrix<T, 3, 3> &covariance);

        virtual ~Point();

      public:
    };

}  // namespace gafro::probabilistic

#include <gafro/probabilistic/Point.hxx>