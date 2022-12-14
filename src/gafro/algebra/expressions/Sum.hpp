/*
    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <gafro/algebra/Multivector.hxx>

namespace gafro
{

    namespace detail
    {
        template <class M1, class M2>
        class SumType
        {
          public:
            using Vtype = typename std::common_type<typename M1::Vtype, typename M2::Vtype>::type;

            template <std::size_t... idx>
            constexpr static auto makeType(std::index_sequence<idx...>)
            {
                return Multivector<Vtype, (M1::bits() | M2::bits()).blades()[idx]...>();
            };

            using Type = decltype(makeType(std::make_index_sequence<(M1::bits() | M2::bits()).size()>{}));
        };
    }  // namespace detail

    template <class M1, class M2>
    class Sum : public BinaryExpression<Sum<M1, M2>, M1, M2, typename detail::SumType<M1, M2>::Type>
    {
      public:
      private:
      public:
        using Type = typename detail::SumType<M1, M2>::Type;
        using Vtype = typename detail::SumType<M1, M2>::Vtype;

        constexpr static int size = Type::size;
        constexpr static auto blades = Type::blades;
        constexpr static auto bits = Type::bits;
        constexpr static auto has = Type::has;

        using Base = BinaryExpression<Sum<M1, M2>, M1, M2, typename detail::SumType<M1, M2>::Type>;

        Sum(const M1 &m1, const M2 &m2) : Base(m1, m2) {}

        virtual ~Sum() = default;

        template <int blade>
        requires(has(blade))  //
          Vtype get()
        const
        {
            if constexpr (M1::has(blade) && M2::has(blade))
            {
                return this->getLeftOperand().template get<blade>() + this->getRightOperand().template get<blade>();
            }
            else if constexpr (M1::has(blade) && !M2::has(blade))
            {
                return this->getLeftOperand().template get<blade>();
            }
            else if constexpr (!M1::has(blade) && M2::has(blade))
            {
                return this->getRightOperand().template get<blade>();
            }
            else
            {
                return 0.0;
            }
        }

      protected:
      private:
    };

    template <typename E1, typename E2>
    requires(E1::isExpression() && E2::isExpression())  //
      Sum<E1, E2>
    operator+(E1 const &u, E2 const &v)
    {
        return Sum<E1, E2>(*static_cast<const E1 *>(&u), *static_cast<const E2 *>(&v));
    }

    template <typename E1, typename E2>
    requires(E1::isExpression() && E2::isExpression())  //
      auto
      operator-(E1 const &u, E2 const &v)
    {
        return u + Scalar<typename E2 ::Vtype>(typename E2 ::Vtype(-1.0)) * v;
    }

}  // namespace gafro