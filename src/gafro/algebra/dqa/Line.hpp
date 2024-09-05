#pragma once

#include <gafro/algebra/dqa/DualQuaternion.hpp>
#include <gafro/algebra/dqa/DualQuaternionAlgebra.hpp>
//
#include <gafro/algebra/cga/Line.hpp>

namespace gafro::dqa
{

    template <class T>
    class Line : public Multivector<T, idx::ek, idx::i, idx::ej, idx::j, idx::ei, idx::k>
    {
      public:
        using Base = Multivector<T, idx::ek, idx::i, idx::ej, idx::j, idx::ei, idx::k>;

        using Base::Base;

        Line(const Base &plane) : Base(plane) {}

        Line(Base &&plane) : Base(std::move(plane)) {}

        Line(const gafro::Line<T> &cga_line)
        {
            this->template set<idx::i>(-cga_line.template get<gafro::blades::e01i>());
            this->template set<idx::j>(cga_line.template get<gafro::blades::e02i>());
            this->template set<idx::k>(cga_line.template get<gafro::blades::e03i>());
            this->template set<idx::ei>(-cga_line.template get<gafro::blades::e23i>());
            this->template set<idx::ej>(cga_line.template get<gafro::blades::e13i>());
            this->template set<idx::ek>(-cga_line.template get<gafro::blades::e12i>());
        }

        Line transform(const DualQuaternion<T> &dq)
        {
            return dq * (*this) * dq.conjugate();
        }

      protected:
      private:
    };

}  // namespace gafro::dqa