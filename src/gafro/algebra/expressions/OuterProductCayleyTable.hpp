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

#include <gafro/algebra/Blades.hpp>
#include <gafro/algebra/Multivector.hxx>

namespace gafro
{
    template <class T, int i1, int i2>
    struct OuterProductCayleyTable
    {
        using Type = Multivector<T>;
    };

    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::scalar>
    {
        using Type = Multivector<T, blades::scalar>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e1>
    {
        using Type = Multivector<T, blades::e1>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e2>
    {
        using Type = Multivector<T, blades::e2>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e3>
    {
        using Type = Multivector<T, blades::e3>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::ei>
    {
        using Type = Multivector<T, blades::ei>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e0>
    {
        using Type = Multivector<T, blades::e0>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e23>
    {
        using Type = Multivector<T, blades::e23>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e13>
    {
        using Type = Multivector<T, blades::e13>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e12>
    {
        using Type = Multivector<T, blades::e12>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e1i>
    {
        using Type = Multivector<T, blades::e1i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e2i>
    {
        using Type = Multivector<T, blades::e2i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e3i>
    {
        using Type = Multivector<T, blades::e3i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e01>
    {
        using Type = Multivector<T, blades::e01>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e02>
    {
        using Type = Multivector<T, blades::e02>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e03>
    {
        using Type = Multivector<T, blades::e03>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e0i>
    {
        using Type = Multivector<T, blades::e0i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e123>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e12i>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e13i>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e23i>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e012>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e013>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e023>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e01i>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e02i>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e03i>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e123i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e0123>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e012i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e023i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e013i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::scalar, blades::e0123i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };

    // /////////////////////////////////////////////////////////
    // e1
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::scalar>
    {
        using Type = Multivector<T, blades::e1>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e2>
    {
        using Type = Multivector<T, blades::e12>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e3>
    {
        using Type = Multivector<T, blades::e13>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::ei>
    {
        using Type = Multivector<T, blades::e1i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e0>
    {
        using Type = Multivector<T, blades::e01>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e23>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e2i>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e3i>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e02>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e03>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e0i>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e23i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e023>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e02i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e03i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e023i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e2
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::scalar>
    {
        using Type = Multivector<T, blades::e2>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e1>
    {
        using Type = Multivector<T, blades::e12>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e3>
    {
        using Type = Multivector<T, blades::e23>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::ei>
    {
        using Type = Multivector<T, blades::e2i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e0>
    {
        using Type = Multivector<T, blades::e02>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e13>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e1i>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e3i>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e01>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e03>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e0i>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e13i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e013>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e01i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e03i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e013i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e3
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::scalar>
    {
        using Type = Multivector<T, blades::e3>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e1>
    {
        using Type = Multivector<T, blades::e13>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e2>
    {
        using Type = Multivector<T, blades::e23>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::ei>
    {
        using Type = Multivector<T, blades::e3i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e0>
    {
        using Type = Multivector<T, blades::e03>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e12>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e1i>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e2i>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e01>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e02>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e0i>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e12i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e012>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e01i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e02i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e012i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // ei
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::scalar>
    {
        using Type = Multivector<T, blades::ei>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e1>
    {
        using Type = Multivector<T, blades::e1i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e2>
    {
        using Type = Multivector<T, blades::e2i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e3>
    {
        using Type = Multivector<T, blades::e3i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e0>
    {
        using Type = Multivector<T, blades::e0i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e23>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e13>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e12>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e01>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e02>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e03>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e123>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e012>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e013>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e023>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e0123>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::ei, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e0
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::scalar>
    {
        using Type = Multivector<T, blades::e0>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e1>
    {
        using Type = Multivector<T, blades::e01>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e2>
    {
        using Type = Multivector<T, blades::e02>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e3>
    {
        using Type = Multivector<T, blades::e03>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::ei>
    {
        using Type = Multivector<T, blades::e0i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e23>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e13>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e12>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e1i>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e2i>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e3i>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e123>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e12i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e13i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e23i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e123i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e23
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::scalar>
    {
        using Type = Multivector<T, blades::e23>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e1>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::ei>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e0>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e1i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e01>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e0i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e01i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e13
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::scalar>
    {
        using Type = Multivector<T, blades::e13>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e2>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::ei>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e0>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e2i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e02>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e0i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e02i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e12
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::scalar>
    {
        using Type = Multivector<T, blades::e12>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e3>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::ei>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e0>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e3i>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e03>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e0i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e03i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e1i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::scalar>
    {
        using Type = Multivector<T, blades::e1i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e2>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e3>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e0>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e23>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e02>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e03>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e023>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e1i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e2i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::scalar>
    {
        using Type = Multivector<T, blades::e2i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e1>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e3>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e0>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e13>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e01>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e03>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e013>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs{ -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e2i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e3i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::scalar>
    {
        using Type = Multivector<T, blades::e3i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e1>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e2>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e0>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e12>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e01>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e02>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e012>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e3i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e01
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::scalar>
    {
        using Type = Multivector<T, blades::e01>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e2>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e3>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::ei>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e23>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e2i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e3i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e23i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e02
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::scalar>
    {
        using Type = Multivector<T, blades::e02>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e1>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e3>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::ei>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e13>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e1i>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e3i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e13i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e03
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::scalar>
    {
        using Type = Multivector<T, blades::e03>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e1>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e2>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::ei>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e12>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e1i>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e2i>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e12i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e0i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::scalar>
    {
        using Type = Multivector<T, blades::e0i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e1>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e2>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e3>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e23>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e13>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e12>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e123>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e123
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::scalar>
    {
        using Type = Multivector<T, blades::e123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::ei>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e0>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e0i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e12i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::scalar>
    {
        using Type = Multivector<T, blades::e12i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e3>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e0>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e03>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e12i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e13i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::scalar>
    {
        using Type = Multivector<T, blades::e13i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e2>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e0>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e02>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e13i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e23i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::scalar>
    {
        using Type = Multivector<T, blades::e23i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e1>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e0>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e01>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e23i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e012
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::scalar>
    {
        using Type = Multivector<T, blades::e012>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e3>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::ei>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e3i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e013
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::scalar>
    {
        using Type = Multivector<T, blades::e013>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e2>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::ei>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e2i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e023
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::scalar>
    {
        using Type = Multivector<T, blades::e023>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e1>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::ei>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e1i>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e01i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::scalar>
    {
        using Type = Multivector<T, blades::e01i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e2>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e3>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e23>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e01i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e02i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::scalar>
    {
        using Type = Multivector<T, blades::e02i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e1>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e3>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e13>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e02i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e03i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::scalar>
    {
        using Type = Multivector<T, blades::e03i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e1>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e2>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e12>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e03i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e123i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::scalar>
    {
        using Type = Multivector<T, blades::e123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e0>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e123i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e0123
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::scalar>
    {
        using Type = Multivector<T, blades::e0123>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::ei>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e012i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::scalar>
    {
        using Type = Multivector<T, blades::e012i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e3>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e012i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e023i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::scalar>
    {
        using Type = Multivector<T, blades::e023i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e1>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { -1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e023i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e013i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::scalar>
    {
        using Type = Multivector<T, blades::e013i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e2>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e013i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };

    // /////////////////////////////////////////////////////////
    // e0123i
    // /////////////////////////////////////////////////////////

    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::scalar>
    {
        using Type = Multivector<T, blades::e0123i>;
        constexpr static std::array<double, Type::size> signs = { 1.0 };
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e1>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e2>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e3>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::ei>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e0>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e23>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e13>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e12>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e1i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e2i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e3i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e01>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e02>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e03>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e0i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e12i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e13i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e23i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e012>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e013>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e023>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e01i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e02i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e03i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e0123>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e012i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e023i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e013i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
    template <class T>
    struct OuterProductCayleyTable<T, blades::e0123i, blades::e0123i>
    {
        using Type = Multivector<T>;
        constexpr static std::array<double, Type::size> signs = {};
    };
}  // namespace gafro