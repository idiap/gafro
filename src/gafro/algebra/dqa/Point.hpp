#pragma once

#include <gafro/algebra/dqa/DualQuaternion.hpp>
#include <gafro/algebra/dqa/DualQuaternionAlgebra.hpp>

namespace gafro::dqa
{

    template <class T>
    class Point : public Multivector<T, idx::scalar, idx::ek, idx::ej, idx::ei>
    {
      public:
        using Base = Multivector<T, idx::scalar, idx::ek, idx::ej, idx::ei>;

        using Base::Base;

        Point(const Base &point) : Base(point) {}

        Point(Base &&point) : Base(std::move(point)) {}

        Point(const DualQuaternion<T> &point) : Base(point.template cast<Base>()) {}

        Point() : Point(0.0, 0.0, 0.0) {}

        Point(const T &x, const T &y, const T &z) : Base({ 1.0, z, y, x }) {}

        Point transform(const DualQuaternion<T> &dq)
        {
            return dq * (*this) * dq.sharpConjugate();
        }

      protected:
      private:
      public:
        static Point Random()
        {
            Base random = Base::Random();
            random.template set<idx::scalar>(1.0);

            return random;
        }
    };

}  // namespace gafro::dqa