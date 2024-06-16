#pragma once

#include <Eigen/Core>
#include <vector>

namespace gafro::geometry
{

    template <class PointType>
    class Pointcloud
    {
      public:
        Pointcloud();

        virtual ~Pointcloud();

        void addPoint(const PointType &point);

        void addPointcloud(const Pointcloud<PointType> &pointcloud);

        void setPoint(const int &i, const PointType &point);

        void downsample(const int &size);

        int getSize() const;

        const PointType &getPoint(const int &i) const;

        virtual Eigen::VectorXd getPointAsVector(const int &i) const = 0;

        const std::vector<PointType> &getPoints() const;

        Eigen::MatrixXd convertToMatrix() const;

      public:
        const PointType &operator[](const int &i) const;

      private:
        std::vector<PointType> points_;
    };

}  // namespace gafro::geometry

#include <gafro/geometry/Pointcloud.hxx>