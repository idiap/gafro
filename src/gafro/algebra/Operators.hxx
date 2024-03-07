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

// #include <gafro/algebra/expressions/Dual.hpp>
// #include <gafro/algebra/expressions/Inverse.hpp>
// #include <gafro/algebra/expressions/Reverse.hpp>
//
#include <gafro/algebra/expressions/GeometricProduct.hpp>
#include <gafro/algebra/expressions/InnerProduct.hpp>
#include <gafro/algebra/expressions/OuterProduct.hpp>
#include <gafro/algebra/expressions/Sum.hpp>
//
#include <gafro/algebra/AbstractMultivector.hpp>
#include <gafro/algebra/expressions/AbstractExpression.hpp>
// #include <gafro/algebra/Multivector.hxx>

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
        return Sum<typename E1::Type, typename E2::Type, detail::AdditionOperator>(std::move(u.derived().evaluate()), std::move(v.derived().evaluate()));
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
    Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator> operator-(const AbstractExpression<E1> &u, const AbstractExpression<E2> &v)
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
        return Sum<typename E1::Type, typename E2::Type, detail::SubstractionOperator>(std::move(u.derived().evaluate()), std::move(v.derived().evaluate()));
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