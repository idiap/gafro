// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/InnerProduct.hpp>
#include <gafro/algebra/OuterProduct.hpp>
#include <gafro/algebra/Sum.hpp>
//
#include <gafro/algebra/AbstractExpression.hpp>
#include <gafro/algebra/AbstractMultivector.hpp>

namespace gafro
{

    // GEOMETRIC PRODUCT

    //-- const expression& * const expression&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, typename E2::Type> operator*(const AbstractExpression<E1> &u, const AbstractExpression<E2> &v)
    {
        return GeometricProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), v.derived().evaluate());
    }

    //-- const expression& * expression&&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, typename E2::Type> operator*(const AbstractExpression<E1> &u, AbstractExpression<E2> &&v)
    {
        return GeometricProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- const expression& * const multivector&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, E2> operator*(const AbstractExpression<E1> &u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<typename E1::Type, E2>(u.derived().evaluate(), v.derived());
    }

    //-- const expression& * multivector&&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, E2> operator*(const AbstractExpression<E1> &u, AbstractMultivector<E2> &&v)
    {
        return GeometricProduct<typename E1::Type, E2>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- expression&& * const expression&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, typename E2::Type> operator*(AbstractExpression<E1> &&u, const AbstractExpression<E2> &v)
    {
        return GeometricProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), v.derived().evaluate());
    }

    //-- expression&& * expression&&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, typename E2::Type> operator*(AbstractExpression<E1> &&u, AbstractExpression<E2> &&v)
    {
        return GeometricProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), std::move(v.derived().evaluate()));
    }

    //-- expression&& * const multivector&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, E2> operator*(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), v.derived());
    }

    //-- expression&& * multivector&&
    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, E2> operator*(AbstractExpression<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return GeometricProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), std::move(v.derived()));
    }

    //-- const multivector& * const expression&
    template <typename E1, typename E2>
    GeometricProduct<E1, typename E2::Type> operator*(const AbstractMultivector<E1> &u, const AbstractExpression<E2> &v)
    {
        return GeometricProduct<E1, typename E2::Type>(u.derived(), v.derived().evaluate());
    }

    //-- const multivector& * expression&&
    template <typename E1, typename E2>
    GeometricProduct<E1, typename E2::Type> operator*(const AbstractMultivector<E1> &u, AbstractExpression<E2> &&v)
    {
        return GeometricProduct<E1, typename E2::Type>(u.derived(), std::move(v.derived().evaluate()));
    }

    //-- const multivector& * const multivector&
    template <typename E1, typename E2>
    GeometricProduct<E1, E2> operator*(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<E1, E2>(u.derived(), v.derived());
    }

    //-- const multivector& * multivector&&
    template <typename E1, typename E2>
    GeometricProduct<E1, E2> operator*(const AbstractMultivector<E1> &u, AbstractMultivector<E2> &&v)
    {
        return GeometricProduct<E1, E2>(u.derived(), std::move(v.derived()));
    }

    //-- multivector&& * const expression&
    template <typename E1, typename E2>
    GeometricProduct<E1, typename E2::Type> operator*(AbstractMultivector<E1> &&u, const AbstractExpression<E2> &v)
    {
        return GeometricProduct<E1, typename E2::Type>(std::move(u.derived()), v.derived().evaluate());
    }

    //-- multivector&& * expression&&
    template <typename E1, typename E2>
    GeometricProduct<E1, typename E2::Type> operator*(AbstractMultivector<E1> &&u, AbstractExpression<E2> &&v)
    {
        return GeometricProduct<E1, typename E2::Type>(std::move(u.derived()), std::move(v.derived().evaluate()));
    }

    //-- multivector&& * const multivector&
    template <typename E1, typename E2>
    GeometricProduct<E1, E2> operator*(AbstractMultivector<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<E1, E2>(std::move(u.derived()), v.derived());
    }

    //-- multivector&& * multivector&&
    template <typename E1, typename E2>
    GeometricProduct<E1, E2> operator*(AbstractMultivector<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return GeometricProduct<E1, E2>(std::move(u.derived()), std::move(v.derived()));
    }

    // DIVISION

    template <typename E1, typename E2>
    GeometricProduct<E1, E2> operator/(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<E1, E2>(u.derived(), v.derived().inverse());
    }

    template <typename E1, typename E2>
    GeometricProduct<typename E1::Type, E2> operator/(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return GeometricProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), v.derived().inverse());
    }

    // SUM

    //-- const expression& + const expression&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator> operator+(const AbstractExpression<E1> &u, const AbstractExpression<E2> &v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator>(u.derived().evaluate(), v.derived().evaluate());
    }

    //-- const expression& + expression&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator> operator+(const AbstractExpression<E1> &u, AbstractExpression<E2> &&v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- const expression& + const multivector&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::AdditionOperator> operator+(const AbstractExpression<E1> &u, const AbstractMultivector<E2> &v)
    {
        return Sum<typename E1::Type, E2, detail::AdditionOperator>(u.derived().evaluate(), v.derived());
    }

    //-- const expression& + multivector&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::AdditionOperator> operator+(const AbstractExpression<E1> &u, AbstractMultivector<E2> &&v)
    {
        return Sum<typename E1::Type, E2, detail::AdditionOperator>(u.derived().evaluate(), std::move(v.derived()));
    }

    //-- expression&& + const expression&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator> operator+(AbstractExpression<E1> &&u, const AbstractExpression<E2> &v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator>(std::move(u.derived().evaluate()), v.derived().evaluate());
    }

    //-- expression&& + expression&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator> operator+(AbstractExpression<E1> &&u, AbstractExpression<E2> &&v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator>(std::move(u.derived().evaluate()),
                                                                                   std::move(v.derived().evaluate()));
    }

    //-- expression&& + const multivector&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::AdditionOperator> operator+(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return Sum<typename E1::Type, E2, detail::AdditionOperator>(std::move(u.derived().evaluate()), v.derived());
    }

    //-- expression&& + multivector&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::AdditionOperator> operator+(AbstractExpression<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return Sum<typename E1::Type, E2, detail::AdditionOperator>(std::move(u.derived().evaluate()), std::move(v.derived()));
    }

    //-- const multivector& + const expression&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::AdditionOperator> operator+(const AbstractMultivector<E1> &u, const AbstractExpression<E2> &v)
    {
        return Sum<E1, typename E2::Type, detail::AdditionOperator>(u.derived(), v.derived().evaluate());
    }

    //-- const multivector& + expression&&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::AdditionOperator> operator+(const AbstractMultivector<E1> &u, AbstractExpression<E2> &&v)
    {
        return Sum<E1, typename E2::Type, detail::AdditionOperator>(u.derived(), std::move(v.derived().evaluate()));
    }

    //-- const multivector& + const multivector&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::AdditionOperator> operator+(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return Sum<E1, E2, detail::AdditionOperator>(u.derived(), v.derived());
    }

    //-- const multivector& + multivector&&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::AdditionOperator> operator+(const AbstractMultivector<E1> &u, AbstractMultivector<E2> &&v)
    {
        return Sum<E1, E2, detail::AdditionOperator>(u.derived(), std::move(v.derived()));
    }

    //-- multivector&& + const expression&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::AdditionOperator> operator+(AbstractMultivector<E1> &&u, const AbstractExpression<E2> &v)
    {
        return Sum<E1, typename E2::Type, detail::AdditionOperator>(std::move(u.derived()), v.derived().evaluate());
    }

    //-- multivector&& + expression&&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::AdditionOperator> operator+(AbstractMultivector<E1> &&u, AbstractExpression<E2> &&v)
    {
        return Sum<E1, typename E2::Type, detail::AdditionOperator>(std::move(u.derived()), std::move(v.derived().evaluate()));
    }

    //-- multivector&& + const multivector&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::AdditionOperator> operator+(AbstractMultivector<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return Sum<E1, E2, detail::AdditionOperator>(std::move(u.derived()), v.derived());
    }

    //-- multivector&& + multivector&&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::AdditionOperator> operator+(AbstractMultivector<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return Sum<E1, E2, detail::AdditionOperator>(std::move(u.derived()), std::move(v.derived()));
    }

    // DIFFERENCE

    //-- const expression& - const expression&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator> operator-(const AbstractExpression<E1> &u,
                                                                                      const AbstractExpression<E2> &v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator>(u.derived().evaluate(), v.derived().evaluate());
    }

    //-- const expression& - expression&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator> operator-(const AbstractExpression<E1> &u, AbstractExpression<E2> &&v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- const expression& - const multivector&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::SubstractionOperator> operator-(const AbstractExpression<E1> &u, const AbstractMultivector<E2> &v)
    {
        return Sum<typename E1::Type, E2, detail::SubstractionOperator>(u.derived().evaluate(), v.derived());
    }

    //-- const expression& - multivector&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::SubstractionOperator> operator-(const AbstractExpression<E1> &u, AbstractMultivector<E2> &&v)
    {
        return Sum<typename E1::Type, E2, detail::SubstractionOperator>(u.derived().evaluate(), std::move(v.derived()));
    }

    //-- expression&& - const expression&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator> operator-(AbstractExpression<E1> &&u, const AbstractExpression<E2> &v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator>(std::move(u.derived().evaluate()), v.derived().evaluate());
    }

    //-- expression&& - expression&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator> operator-(AbstractExpression<E1> &&u, AbstractExpression<E2> &&v)
    {
        return Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator>(std::move(u.derived().evaluate()),
                                                                                       std::move(v.derived().evaluate()));
    }

    //-- expression&& - const multivector&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::SubstractionOperator> operator-(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return Sum<typename E1::Type, E2, detail::SubstractionOperator>(std::move(u.derived().evaluate()), v.derived());
    }

    //-- expression&& - multivector&&
    template <typename E1, typename E2>
    Sum<typename E1::Type, E2, detail::SubstractionOperator> operator-(AbstractExpression<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return Sum<typename E1::Type, E2, detail::SubstractionOperator>(std::move(u.derived().evaluate()), std::move(v.derived()));
    }

    //-- const multivector& - const expression&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::SubstractionOperator> operator-(const AbstractMultivector<E1> &u, const AbstractExpression<E2> &v)
    {
        return Sum<E1, typename E2::Type, detail::SubstractionOperator>(u.derived(), v.derived().evaluate());
    }

    //-- const multivector& - expression&&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::SubstractionOperator> operator-(const AbstractMultivector<E1> &u, AbstractExpression<E2> &&v)
    {
        return Sum<E1, typename E2::Type, detail::SubstractionOperator>(u.derived(), std::move(v.derived().evaluate()));
    }

    //-- const multivector& - const multivector&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::SubstractionOperator> operator-(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return Sum<E1, E2, detail::SubstractionOperator>(u.derived(), v.derived());
    }

    //-- const multivector& - multivector&&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::SubstractionOperator> operator-(const AbstractMultivector<E1> &u, AbstractMultivector<E2> &&v)
    {
        return Sum<E1, E2, detail::SubstractionOperator>(u.derived(), std::move(v.derived()));
    }

    //-- multivector&& - const expression&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::SubstractionOperator> operator-(AbstractMultivector<E1> &&u, const AbstractExpression<E2> &v)
    {
        return Sum<E1, typename E2::Type, detail::SubstractionOperator>(std::move(u.derived()), v.derived().evaluate());
    }

    //-- multivector&& - expression&&
    template <typename E1, typename E2>
    Sum<E1, typename E2::Type, detail::SubstractionOperator> operator-(AbstractMultivector<E1> &&u, AbstractExpression<E2> &&v)
    {
        return Sum<E1, typename E2::Type, detail::SubstractionOperator>(std::move(u.derived()), std::move(v.derived().evaluate()));
    }

    //-- multivector&& - const multivector&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::SubstractionOperator> operator-(AbstractMultivector<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return Sum<E1, E2, detail::SubstractionOperator>(std::move(u.derived()), v.derived());
    }

    //-- multivector&& - multivector&&
    template <typename E1, typename E2>
    Sum<E1, E2, detail::SubstractionOperator> operator-(AbstractMultivector<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return Sum<E1, E2, detail::SubstractionOperator>(std::move(u.derived()), std::move(v.derived()));
    }

    // INNER PRODUCT

    //-- const expression& | const expression&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, typename E2::Type> operator|(const AbstractExpression<E1> &u, const AbstractExpression<E2> &v)
    {
        return InnerProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), v.derived().evaluate());
    }

    //-- const expression& | expression&&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, typename E2::Type> operator|(const AbstractExpression<E1> &u, AbstractExpression<E2> &&v)
    {
        return InnerProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- const expression& | const multivector&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, E2> operator|(const AbstractExpression<E1> &u, const AbstractMultivector<E2> &v)
    {
        return InnerProduct<typename E1::Type, E2>(u.derived().evaluate(), v.derived());
    }

    //-- const expression& | multivector&&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, E2> operator|(const AbstractExpression<E1> &u, AbstractMultivector<E2> &&v)
    {
        return InnerProduct<typename E1::Type, E2>(u.derived().evaluate(), std::move(v.derived()));
    }

    //-- expression&& | const expression&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, typename E2::Type> operator|(AbstractExpression<E1> &&u, const AbstractExpression<E2> &v)
    {
        return InnerProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), v.derived().evaluate());
    }

    //-- expression&& | expression&&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, typename E2::Type> operator|(AbstractExpression<E1> &&u, AbstractExpression<E2> &&v)
    {
        return InnerProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), std::move(v.derived().evaluate()));
    }

    //-- expression&& | const multivector&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, E2> operator|(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return InnerProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), v.derived());
    }

    //-- expression&& | multivector&&
    template <typename E1, typename E2>
    InnerProduct<typename E1::Type, E2> operator|(AbstractExpression<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return InnerProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), std::move(v.derived()));
    }

    //-- const multivector& | const expression&
    template <typename E1, typename E2>
    InnerProduct<E1, typename E2::Type> operator|(const AbstractMultivector<E1> &u, const AbstractExpression<E2> &v)
    {
        return InnerProduct<E1, typename E2::Type>(u.derived(), v.derived().evaluate());
    }

    //-- const multivector& | expression&&
    template <typename E1, typename E2>
    InnerProduct<E1, typename E2::Type> operator|(const AbstractMultivector<E1> &u, AbstractExpression<E2> &&v)
    {
        return InnerProduct<E1, typename E2::Type>(u.derived(), std::move(v.derived().evaluate()));
    }

    //-- const multivector& | const multivector&
    template <typename E1, typename E2>
    InnerProduct<E1, E2> operator|(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return InnerProduct<E1, E2>(u.derived(), v.derived());
    }

    //-- const multivector& | multivector&&
    template <typename E1, typename E2>
    InnerProduct<E1, E2> operator|(const AbstractMultivector<E1> &u, AbstractMultivector<E2> &&v)
    {
        return InnerProduct<E1, E2>(u.derived(), std::move(v.derived()));
    }

    //-- multivector&& | const expression&
    template <typename E1, typename E2>
    InnerProduct<E1, typename E2::Type> operator|(AbstractMultivector<E1> &&u, const AbstractExpression<E2> &v)
    {
        return InnerProduct<E1, typename E2::Type>(std::move(u.derived()), v.derived().evaluate());
    }

    //-- multivector&& | expression&&
    template <typename E1, typename E2>
    InnerProduct<E1, typename E2::Type> operator|(AbstractMultivector<E1> &&u, AbstractExpression<E2> &&v)
    {
        return InnerProduct<E1, typename E2::Type>(std::move(u.derived()), std::move(v.derived().evaluate()));
    }

    //-- multivector&& | const multivector&
    template <typename E1, typename E2>
    InnerProduct<E1, E2> operator|(AbstractMultivector<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return InnerProduct<E1, E2>(std::move(u.derived()), v.derived());
    }

    //-- multivector&& | multivector&&
    template <typename E1, typename E2>
    InnerProduct<E1, E2> operator|(AbstractMultivector<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return InnerProduct<E1, E2>(std::move(u.derived()), std::move(v.derived()));
    }

    // OUTER PRODUCT

    //-- const expression& ^ const expression&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, typename E2::Type> operator^(const AbstractExpression<E1> &u, const AbstractExpression<E2> &v)
    {
        return OuterProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), v.derived().evaluate());
    }

    //-- const expression& ^ expression&&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, typename E2::Type> operator^(const AbstractExpression<E1> &u, AbstractExpression<E2> &&v)
    {
        return OuterProduct<typename E1::Type, typename E2::Type>(u.derived().evaluate(), std::move(v.derived().evaluate()));
    }

    //-- const expression& ^ const multivector&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, E2> operator^(const AbstractExpression<E1> &u, const AbstractMultivector<E2> &v)
    {
        return OuterProduct<typename E1::Type, E2>(u.derived().evaluate(), v.derived());
    }

    //-- const expression& ^ multivector&&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, E2> operator^(const AbstractExpression<E1> &u, AbstractMultivector<E2> &&v)
    {
        return OuterProduct<typename E1::Type, E2>(u.derived().evaluate(), std::move(v.derived()));
    }

    //-- expression&& ^ const expression&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, typename E2::Type> operator^(AbstractExpression<E1> &&u, const AbstractExpression<E2> &v)
    {
        return OuterProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), v.derived().evaluate());
    }

    //-- expression&& ^ expression&&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, typename E2::Type> operator^(AbstractExpression<E1> &&u, AbstractExpression<E2> &&v)
    {
        return OuterProduct<typename E1::Type, typename E2::Type>(std::move(u.derived().evaluate()), std::move(v.derived().evaluate()));
    }

    //-- expression&& ^ const multivector&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, E2> operator^(AbstractExpression<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return OuterProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), v.derived());
    }

    //-- expression&& ^ multivector&&
    template <typename E1, typename E2>
    OuterProduct<typename E1::Type, E2> operator^(AbstractExpression<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return OuterProduct<typename E1::Type, E2>(std::move(u.derived().evaluate()), std::move(v.derived()));
    }

    //-- const multivector& ^ const expression&
    template <typename E1, typename E2>
    OuterProduct<E1, typename E2::Type> operator^(const AbstractMultivector<E1> &u, const AbstractExpression<E2> &v)
    {
        return OuterProduct<E1, typename E2::Type>(u.derived(), v.derived().evaluate());
    }

    //-- const multivector& ^ expression&&
    template <typename E1, typename E2>
    OuterProduct<E1, typename E2::Type> operator^(const AbstractMultivector<E1> &u, AbstractExpression<E2> &&v)
    {
        return OuterProduct<E1, typename E2::Type>(u.derived(), std::move(v.derived().evaluate()));
    }

    //-- const multivector& ^ const multivector&
    template <typename E1, typename E2>
    OuterProduct<E1, E2> operator^(const AbstractMultivector<E1> &u, const AbstractMultivector<E2> &v)
    {
        return OuterProduct<E1, E2>(u.derived(), v.derived());
    }

    //-- const multivector& ^ multivector&&
    template <typename E1, typename E2>
    OuterProduct<E1, E2> operator^(const AbstractMultivector<E1> &u, AbstractMultivector<E2> &&v)
    {
        return OuterProduct<E1, E2>(u.derived(), std::move(v.derived()));
    }

    //-- multivector&& ^ const expression&
    template <typename E1, typename E2>
    OuterProduct<E1, typename E2::Type> operator^(AbstractMultivector<E1> &&u, const AbstractExpression<E2> &v)
    {
        return OuterProduct<E1, typename E2::Type>(std::move(u.derived()), v.derived().evaluate());
    }

    //-- multivector&& ^ expression&&
    template <typename E1, typename E2>
    OuterProduct<E1, typename E2::Type> operator^(AbstractMultivector<E1> &&u, AbstractExpression<E2> &&v)
    {
        return OuterProduct<E1, typename E2::Type>(std::move(u.derived()), std::move(v.derived().evaluate()));
    }

    //-- multivector&& ^ const multivector&
    template <typename E1, typename E2>
    OuterProduct<E1, E2> operator^(AbstractMultivector<E1> &&u, const AbstractMultivector<E2> &v)
    {
        return OuterProduct<E1, E2>(std::move(u.derived()), v.derived());
    }

    //-- multivector&& ^ multivector&&
    template <typename E1, typename E2>
    OuterProduct<E1, E2> operator^(AbstractMultivector<E1> &&u, AbstractMultivector<E2> &&v)
    {
        return OuterProduct<E1, E2>(std::move(u.derived()), std::move(v.derived()));
    }

}  // namespace gafro