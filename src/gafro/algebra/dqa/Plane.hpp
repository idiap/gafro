#pragma once

#include <gafro/algebra/dqa/DualQuaternion.hpp>
#include <gafro/algebra/dqa/DualQuaternionAlgebra.hpp>
//
#include <gafro/algebra/cga/Plane.hpp>

namespace gafro::dqa
{

    template <class T>
    class Plane : public Multivector<T, idx::i, idx::j, idx::k, idx::e>
    {
      public:
        using Base = Multivector<T, idx::i, idx::j, idx::k, idx::e>;

        using Base::Base;

        Plane(const Base &plane) : Base(plane) {}

        Plane(Base &&plane) : Base(std::move(plane)) {}

        Plane(const T &x, const T &y, const T &z, const T &d) : Base({ x, y, z, d }) {}

        Plane(const gafro::Plane<T> &cga_plane)
        {
            this->template set<idx::i>(-cga_plane.template get<gafro::blades::e023i>());
            this->template set<idx::j>(-cga_plane.template get<gafro::blades::e013i>());
            this->template set<idx::k>(cga_plane.template get<gafro::blades::e012i>());
            this->template set<idx::e>(cga_plane.template get<gafro::blades::e123i>());
        }

        Plane transform(const DualQuaternion<T> &dq)
        {
            return dq.sharp() * (*this) * dq.conjugate();
        }

        Eigen::Vector3<T> getNormal() const
        {
            return Eigen::Vector3d({ this->template get<idx::i>(), this->template get<idx::j>(), this->template get<idx::k>() });
        }

        Plane normalized() const
        {
            Eigen::Vector3d normal = getNormal().normalized();

            Plane normalized_plane;

            normalized_plane.template set<idx::i>(normal.x());
            normalized_plane.template set<idx::j>(normal.y());
            normalized_plane.template set<idx::k>(normal.z());
            normalized_plane.template set<idx::e>(this->template get<idx::e>());

            return normalized_plane;
        }

        T getDistance(const Point<T> &point) const
        {
            return (DualQuaternion<T>(point).getDual().toPrimal().getImaginary().anticommute(DualQuaternion<T>(*this).getPrimal()) -
                    DualQuaternion<T>(*this).getDual().toPrimal())
              .template get<0>();
        }

        DualQuaternion<T> composeTransformation(const Plane &other)
        {
            return (*this) * DualQuaternion<T>(other).sharp();
        }

      protected:
      private:
    };

}  // namespace gafro::dqa