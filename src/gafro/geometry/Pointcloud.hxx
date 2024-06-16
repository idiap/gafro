#pragma once

#include <gafro/geometry/Pointcloud.hpp>

namespace gafro::geometry
{

    template <class PointType>
    Pointcloud<PointType>::Pointcloud() = default;

    template <class PointType>
    Pointcloud<PointType>::~Pointcloud() = default;

    template <class PointType>
    void Pointcloud<PointType>::setPoint(const int &i, const PointType &point)
    {
        points_[i] = point;
    }

    template <class PointType>
    void Pointcloud<PointType>::addPoint(const PointType &point)
    {
        points_.push_back(point);
    }

    template <class PointType>
    void Pointcloud<PointType>::addPointcloud(const Pointcloud<PointType> &pointcloud)
    {
        for (unsigned i = 0; i < pointcloud.getSize(); ++i)
        {
            points_.push_back(pointcloud.getPoint(i));
        }
    }

    template <class PointType>
    void Pointcloud<PointType>::downsample(const int &size)
    {
        double threshold = static_cast<double>(size) / static_cast<double>(getSize());

        std::vector<PointType> points;

        while (points.size() < size)
        {
            points.push_back(getPoint(std::rand() % getSize()));
        }

        points_ = std::move(points);
    }

    template <class PointType>
    int Pointcloud<PointType>::getSize() const
    {
        return points_.size();
    }

    template <class PointType>
    const PointType &Pointcloud<PointType>::getPoint(const int &i) const
    {
        return points_[i];
    }

    template <class PointType>
    const std::vector<PointType> &Pointcloud<PointType>::getPoints() const
    {
        return points_;
    }

    template <class PointType>
    Eigen::MatrixXd Pointcloud<PointType>::convertToMatrix() const
    {
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(getPointAsVector(0).rows(), points_.size());

        for (int i = 0; i < points_.size(); ++i)
        {
            matrix.col(i) = getPointAsVector(i);
        }

        return matrix;
    }

    template <class PointType>
    const PointType &Pointcloud<PointType>::operator[](const int &i) const
    {
        return getPoint(i);
    }

}  // namespace gafro::geometry