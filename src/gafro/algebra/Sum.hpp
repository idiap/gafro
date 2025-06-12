// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/BinaryExpression.hpp>
#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/TypeTraits.hpp>

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
                return typename M1::MAlgebra::template Multivector<Vtype, (M1::bits() | M2::bits()).blades()[idx]...>();
            }

            using Type = decltype(makeType(std::make_index_sequence<(M1::bits() | M2::bits()).size()>{}));
        };

        template <class T>
        class AdditionOperator
        {
          public:
            T operator()(const T &lhs, const T &rhs)
            {
                return lhs + rhs;
            }
        };

        template <class T>
        class SubstractionOperator
        {
          public:
            T operator()(const T &lhs, const T &rhs)
            {
                return lhs - rhs;
            }
        };

    }  // namespace detail

    template <class M1, class M2, template <class V> class Operation>
    class Sum : public BinaryExpression<Sum<M1, M2, Operation>, M1, M2, typename detail::SumType<M1, M2>::Type>
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

        using Base = BinaryExpression<Sum<M1, M2, Operation>, M1, M2, typename detail::SumType<M1, M2>::Type>;

        Sum(const M1 &m1, const M2 &m2)  //
          : Base(m1, m2)
        {}

        Sum(M1 &&m1, M2 &&m2)  //
          : Base(std::move(m1), std::move(m2))
        {}

        Sum(const M1 &m1, M2 &&m2)  //
          : Base(m1, std::move(m2))
        {}

        Sum(M1 &&m1, const M2 &m2)  //
          : Base(std::move(m1), m2)
        {}

        virtual ~Sum() = default;

        template <int blade>
            requires(has(blade))  //
        inline Vtype get() const
        {
            if constexpr (M1::has(blade) && M2::has(blade))
            {
                return Operation<Vtype>()(this->getLeftOperand().template get<blade>(), this->getRightOperand().template get<blade>());
            }
            else if constexpr (M1::has(blade) && !M2::has(blade))
            {
                return Operation<Vtype>()(this->getLeftOperand().template get<blade>(), TypeTraits<Vtype>::Zero());
            }
            else if constexpr (!M1::has(blade) && M2::has(blade))
            {
                return Operation<Vtype>()(TypeTraits<Vtype>::Zero(), this->getRightOperand().template get<blade>());
            }
            else
            {
                return TypeTraits<Vtype>::Zero();
            }
        }

      protected:
      private:
    };

}  // namespace gafro