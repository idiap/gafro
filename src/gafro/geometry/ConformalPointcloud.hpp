#pragma once

#include <gafro/geometry/EuclideanPointcloud.hpp>
#include <gafro/geometry/Pointcloud.hpp>
//
#include <gafro/algebra/cga/Motor.hpp>
#include <gafro/algebra/cga/Point.hpp>

namespace gafro::geometry
{

    template <class T>
    class ConformalPointcloud : public Pointcloud<Point<T>>
    {
      public:
        ConformalPointcloud();

        ConformalPointcloud(const EuclideanPointcloud<T> &pointcloud);

        virtual ~ConformalPointcloud();

        Eigen::VectorXd getPointAsVector(const int &i) const;

        ConformalPointcloud transform(const Motor<T> &motor) const;

      protected:
      private:
    };

}  // namespace gafro::geometry

#include <gafro/geometry/ConformalPointcloud.hxx>