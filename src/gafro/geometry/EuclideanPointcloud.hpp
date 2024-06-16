#pragma once

#include <Eigen/Core>
#include <gafro/geometry/Pointcloud.hpp>

namespace gafro::geometry
{

    template <class T>
    class ConformalPointcloud;

    template <class T>
    class EuclideanPointcloud : public Pointcloud<Eigen::Vector3<T>>
    {
      public:
        EuclideanPointcloud();

        virtual ~EuclideanPointcloud();

        ConformalPointcloud<T> conformalize() const;

        Eigen::VectorXd getPointAsVector(const int &i) const;

      protected:
      private:
      public:
        static EuclideanPointcloud createFromMatrix(const Eigen::Matrix<T, 3, Eigen::Dynamic> &matrix);

        static EuclideanPointcloud createSphericalPointcloud(const double &radius, const int &size);

        static EuclideanPointcloud createGridPointcloud(const Eigen::Vector3<T> &center, const double &width, const double &height,
                                                        const double &depth, const int &size);

        static EuclideanPointcloud createPlanePointcloud(const Eigen::Vector3<T> &center, const double &width, const double &height, const int &size);
    };

}  // namespace gafro::geometry

#include <gafro/geometry/EuclideanPointcloud.hxx>