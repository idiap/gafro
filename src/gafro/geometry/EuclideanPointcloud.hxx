#pragma once

#include <gafro/geometry/EuclideanPointcloud.hpp>
//
#include <gafro/geometry/ConformalPointcloud.hpp>

namespace gafro::geometry
{

    template <class T>
    EuclideanPointcloud<T>::EuclideanPointcloud() = default;

    template <class T>
    EuclideanPointcloud<T>::~EuclideanPointcloud() = default;

    template <class T>
    ConformalPointcloud<T> EuclideanPointcloud<T>::conformalize() const
    {
        return ConformalPointcloud<T>(*this);
    }

    template <class T>
    Eigen::VectorXd EuclideanPointcloud<T>::getPointAsVector(const int &i) const
    {
        return this->getPoint(i).template cast<double>();
    }

    template <class T>
    EuclideanPointcloud<T> EuclideanPointcloud<T>::createFromMatrix(const Eigen::Matrix<T, 3, Eigen::Dynamic> &matrix)
    {
        EuclideanPointcloud<T> pointcloud;

        for (int i = 0; i < matrix.cols(); ++i)
        {
            pointcloud.addPoint(matrix.col(i));
        }

        return pointcloud;
    }

    template <class T>
    EuclideanPointcloud<T> EuclideanPointcloud<T>::createSphericalPointcloud(const double &radius, const int &size)
    {
        Eigen::VectorX<T> z = Eigen::VectorX<T>::Random(size).array().abs();
        Eigen::VectorX<T> phi = M_PI * Eigen::VectorX<T>::Random(size);

        Eigen::VectorX<T> x = (1.0 - z.array().square()).sqrt() * phi.array().cos();
        Eigen::VectorX<T> y = (1.0 - z.array().square()).sqrt() * phi.array().sin();

        Eigen::Matrix<T, 3, Eigen::Dynamic> pointcloud = Eigen::MatrixX<T>::Zero(3, size);

        pointcloud.row(0) = radius * x.transpose();
        pointcloud.row(1) = radius * y.transpose();
        pointcloud.row(2) = radius * z.transpose();

        return createFromMatrix(pointcloud);
    }

    template <class T>
    EuclideanPointcloud<T> EuclideanPointcloud<T>::createGridPointcloud(const Eigen::Vector3<T> &center, const double &width, const double &depth,
                                                                        const double &height, const int &size)
    {
        double increment = std::pow(static_cast<double>(size), 1.0 / 3.0);

        EuclideanPointcloud<T> pointcloud;

        for (double x = center.x() - 0.5 * width; x < center.x() + 0.5 * width; x += width / increment)
        {
            for (double y = center.y() - 0.5 * depth; y < center.y() + 0.5 * depth; y += depth / increment)
            {
                for (double z = center.z() - 0.5 * height; z < center.z() + 0.5 * height; z += height / increment)
                {
                    pointcloud.addPoint(Eigen::Vector3<T>({ x, y, z }));
                }
            }
        }

        pointcloud.downsample(size);

        return pointcloud;
    }

    template <class T>
    EuclideanPointcloud<T> EuclideanPointcloud<T>::createPlanePointcloud(const Eigen::Vector3<T> &center, const double &width, const double &height,
                                                                         const int &size)
    {
        Eigen::VectorX<T> x = 0.5 * width * Eigen::VectorX<T>::Random(size);
        Eigen::VectorX<T> y = 0.5 * height * Eigen::VectorX<T>::Random(size);

        Eigen::Matrix<T, 3, Eigen::Dynamic> pointcloud = Eigen::MatrixX<T>::Zero(3, size);

        pointcloud.row(0) = x.transpose();
        pointcloud.row(1) = y.transpose();

        return createFromMatrix(pointcloud);
    }

}  // namespace gafro::geometry