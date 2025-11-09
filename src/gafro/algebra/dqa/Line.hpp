// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

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

        Line(const Base &plane)
          : Base(plane)
        {}

        Line(Base &&plane)
          : Base(std::move(plane))
        {}

        Line(const typename Base::Parameters &plane)
          : Base(plane)
        {}

        Line(const gafro::Line<T> &cga_line)
        {
            auto dual = cga_line.dual();

            this->template set<idx::i>(dual.template get<gafro::blades::e23>());
            this->template set<idx::j>(-dual.template get<gafro::blades::e13>());
            this->template set<idx::k>(dual.template get<gafro::blades::e12>());
            this->template set<idx::ei>(dual.template get<gafro::blades::e1i>());
            this->template set<idx::ej>(dual.template get<gafro::blades::e2i>());
            this->template set<idx::ek>(dual.template get<gafro::blades::e3i>());
        }

        Line transform(const DualQuaternion<T> &dq)
        {
            return dq * (*this) * dq.conjugate();
        }

        operator gafro::Line<T>() const
        {
            gafro::Line<T> line;

            line.template set<gafro::blades::e01i>(this->template get<idx::i>());
            line.template set<gafro::blades::e02i>(this->template get<idx::j>());
            line.template set<gafro::blades::e03i>(this->template get<idx::k>());
            line.template set<gafro::blades::e23i>(this->template get<idx::ei>());
            line.template set<gafro::blades::e13i>(this->template get<idx::ej>());
            line.template set<gafro::blades::e12i>(this->template get<idx::ek>());

            return line;
        }

      protected:
      private:
    };

}  // namespace gafro::dqa