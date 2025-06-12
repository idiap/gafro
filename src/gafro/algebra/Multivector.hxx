// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra/Assign.hpp>
#include <gafro/algebra/Cast.hpp>
#include <gafro/algebra/CommutatorProduct.hpp>
#include <gafro/algebra/Conjugate.hpp>
#include <gafro/algebra/Dual.hpp>
#include <gafro/algebra/DualPoincare.hpp>
#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/InnerProduct.hpp>
#include <gafro/algebra/Inverse.hpp>
#include <gafro/algebra/OuterProduct.hpp>
#include <gafro/algebra/Reverse.hpp>
#include <gafro/algebra/SharpConjugate.hpp>
#include <gafro/algebra/Sum.hpp>
//
#include <gafro/algebra/Multivector.hpp>

namespace gafro
{

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector() : Multivector(Parameters::Constant(TypeTraits<T>::Zero()))
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const T &value)
        requires(sizeof...(index) == 1)
      : Multivector(Parameters::Constant(value))
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const Parameters &parameters) : parameters_(parameters)
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(Parameters &&parameters) : parameters_(std::move(parameters))
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(const Multivector &other) : parameters_(other.parameters_)
    {}

    template <class M>
    template <class T, int... index>
    Algebra<M>::Multivector<T, index...>::Multivector(Multivector &&other) : parameters_(std::move(other.parameters_))
    {}

    template <class M>
    template <class T, int... index>
    template <class Derived>
    Algebra<M>::Multivector<T, index...>::Multivector(const Expression<Derived, Multivector> &expression)
    {
        *this = expression;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived, class Other>
    Algebra<M>::Multivector<T, index...>::Multivector(const Expression<Derived, Other> &expression)
    {
        *this = expression;
    }

    template <class M>
    template <class T, int... index>
    template <class S>
    Algebra<M>::Multivector<T, index...>::Multivector(const typename Algebra<M>::template Multivector<S, index...> &other)
      : parameters_(other.vector().unaryExpr([](const S &v) { return TypeTraits<T>::Value(v); }))
    {}

    // OPERATORS

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::setParameters(Parameters &&parameters)
    {
        parameters_.noalias() = std::move(parameters);
    }

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::setParameters(const Parameters &parameters)
    {
        parameters_.noalias() = parameters;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...>::Parameters &Algebra<M>::Multivector<T, index...>::vector()
    {
        return parameters_;
    }

    template <class M>
    template <class T, int... index>
    const typename Algebra<M>::template Multivector<T, index...>::Parameters &Algebra<M>::Multivector<T, index...>::vector() const
    {
        return parameters_;
    }

    //

    template <class M>
    template <class T, int... index>
    Reverse<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::reverse() const
    {
        return Reverse<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    Conjugate<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::conjugate() const
    {
        return Conjugate<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    SharpConjugate<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::sharpConjugate() const
    {
        return SharpConjugate<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    Inverse<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::inverse() const
    {
        return Inverse<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    Dual<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::dual() const
    {
        return Dual<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    template <class M>
    template <class T, int... index>
    DualPoincare<typename Algebra<M>::template Multivector<T, index...>> Algebra<M>::Multivector<T, index...>::dualPoincare() const
    {
        return DualPoincare<typename Algebra<M>::template Multivector<T, index...>>(*this);
    }

    // OPERATORS

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(const Parameters &parameters)
    {
        setParameters(parameters);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(Parameters &&parameters)
    {
        setParameters(std::move(parameters));

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(const Multivector &other)
    {
        setParameters(other.parameters_);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(Multivector &&other)
    {
        setParameters(std::move(other.parameters_));

        return *this;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(
      const Expression<Derived, Multivector> &expression)
    {
        Assign<Derived, Multivector, T, M, index...>::run(*this, expression);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    template <class Derived, class Other>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator=(
      const Expression<Derived, Other> &expression)
    {
        Assign<Derived, Other, T, M, index...>::run(*this, expression);

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator*=(const T &scalar)
    {
        parameters_ *= scalar;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator/=(const T &scalar)
    {
        parameters_ /= scalar;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> &Algebra<M>::Multivector<T, index...>::operator+=(const Multivector &other)
    {
        parameters_ += other.parameters_;

        return *this;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::operator-() const
    {
        return Multivector(-this->vector());
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::operator*(const T &scalar) const
    {
        typename Algebra<M>::template Scalar<T> s;

        s.template set<0>(scalar);

        return (*this) * s;
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::operator/(const T &scalar) const
    {
        return (*this) * (TypeTraits<T>::Value(1.0) / scalar);
    }

    template <class M>
    template <class T, int... index>
    template <template <class S> class MType, int rows, int cols>
    auto Algebra<M>::Multivector<T, index...>::operator*(const MultivectorMatrix<T, MType, rows, cols> &matrix) const
    {
        using GP = gafro::GeometricProduct<Multivector, MType<T>>;
        typename GP::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, (*this) * matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    template <class M>
    template <class T, int... index>
    template <template <class S> class MType, int rows, int cols>
    auto Algebra<M>::Multivector<T, index...>::operator|(const MultivectorMatrix<T, MType, rows, cols> &matrix) const
    {
        using IP = gafro::InnerProduct<Multivector, MType<T>>;
        typename IP::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, (*this) | matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    template <class M>
    template <class T, int... index>
    template <template <class S> class MType, int rows, int cols>
    auto Algebra<M>::Multivector<T, index...>::operator^(const MultivectorMatrix<T, MType, rows, cols> &matrix) const
    {
        using OP = gafro::OuterProduct<Multivector, MType<T>>;
        typename OP::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, (*this) ^ matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    template <class M>
    template <class T, int... index>
    template <template <class S> class MType, int rows, int cols>
    auto Algebra<M>::Multivector<T, index...>::operator+(const MultivectorMatrix<T, MType, rows, cols> &matrix) const
    {
        using Summation = gafro::Sum<Multivector, MType<T>, gafro::detail::AdditionOperator>;
        typename Summation::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, (*this) + matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    template <class M>
    template <class T, int... index>
    template <template <class S> class MType, int rows, int cols>
    auto Algebra<M>::Multivector<T, index...>::operator-(const MultivectorMatrix<T, MType, rows, cols> &matrix) const
    {
        using Substraction = gafro::Sum<Multivector, MType<T>, gafro::detail::SubstractionOperator>;
        typename Substraction::Type::template Matrix<rows, cols> result;

        for (unsigned r = 0; r < rows; r++)
        {
            for (unsigned c = 0; c < cols; c++)
            {
                result.setCoefficient(r, c, (*this) - matrix.getCoefficient(r, c));
            }
        }

        return result;
    }

    //

    template <class M>
    template <class T, int... index>
    constexpr const Bitset<Algebra<M>::dim, index...> &Algebra<M>::Multivector<T, index...>::bits()
    {
        return bits_;
    }

    template <class M>
    template <class T, int... index>
    constexpr const std::array<int, Algebra<M>::template Multivector<T, index...>::size> &Algebra<M>::Multivector<T, index...>::blades()
    {
        return blades_;
    }

    template <class M>
    template <class T, int... index>
    constexpr bool Algebra<M>::Multivector<T, index...>::has(const int &blade)
    {
        return bits_.test(blade);
    }

    //

    template <class M>
    template <class T, int... index>
    template <class M2>
    auto Algebra<M>::Multivector<T, index...>::commute(const M2 &multivector) const
    {
        return Scalar<T>((Eigen::Matrix<T, 1, 1>() << TypeTraits<T>::Value(0.5)).finished()) * ((*this) * multivector - multivector * (*this));
    }

    template <class M>
    template <class T, int... index>
    template <class M2>
    auto Algebra<M>::Multivector<T, index...>::anticommute(const M2 &multivector) const
    {
        return Scalar<T>((Eigen::Matrix<T, 1, 1>() << TypeTraits<T>::Value(0.5)).finished()) * ((*this) * multivector + multivector * (*this));
    }

    //

    // template <class M>
    // template <class T, int... index>
    // template <int blade>
    //     requires(Algebra<M>::template Multivector<T, index...>::has(blade))  //
    // typename Algebra<M>::template Multivector<T, blade> Algebra<M>::template Multivector<T, index...>::getBlade() const
    // {
    //     return typename Algebra<M>::template Multivector<T, blade>(this->template get<blade>());
    // }

    //

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::norm() const
    {
        return sqrt(abs(squaredNorm()));
    }

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::squaredNorm() const
    {
        return ((*this) * this->reverse()).template get<0>();
    }

    template <class M>
    template <class T, int... index>
    T Algebra<M>::Multivector<T, index...>::signedNorm() const
    {
        T squared_norm = squaredNorm();

        return squared_norm > 0 ? sqrt(squared_norm) : -sqrt(abs(squared_norm));
    }

    template <class M>
    template <class T, int... index>
    void Algebra<M>::Multivector<T, index...>::normalize()
    {
        T value = norm();

        // if (value > 1e-10)
        // {
        *this = (*this) / value;  // Algebra<M>::Scalar<T>(typename Algebra<M>::Scalar<T>::Parameters({ 1.0 / value }));
        // }
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::normalized() const
    {
        Multivector other = *this;

        other.normalize();

        return other;
    }

    //

    template <class M>
    template <class T, int... index>
    auto Algebra<M>::Multivector<T, index...>::square() const
    {
        return (*this) * (*this);
    }

    //

    template <class M>
    template <class T, int... index>
    template <std::size_t... i>
    constexpr auto Algebra<M>::Multivector<T, index...>::getBlades(std::index_sequence<i...>, const Multivector &multivector)
    {
        return std::tuple{ Blade<T, blades()[i]>(multivector.get<blades()[i]>())... };
    }

    template <class M>
    template <class T, int... index>
    auto Algebra<M>::Multivector<T, index...>::getBlades() const
    {
        return getBlades(std::make_index_sequence<size>{}, *this);
    }

    //

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::Random()
    {
        return Multivector(Parameters::Random());
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::Zero()
    {
        return Multivector(Parameters::Constant(TypeTraits<T>::Zero()));
    }

    template <class M>
    template <class T, int... index>
    typename Algebra<M>::template Multivector<T, index...> Algebra<M>::Multivector<T, index...>::One()
    {
        return Multivector(Parameters::Ones());
    }

    template <class M>
    template <class T, int... index>
    template <class Other>
    Other Algebra<M>::Multivector<T, index...>::cast() const
    {
        return Cast<typename Algebra<M>::template Multivector<T, index...>, Other>(*this);
    }

    template <class T, class Derived,
              class = typename std::enable_if<TypeTraits<T>::is_scalar_type>::type>  //
    Derived operator*(const T &scalar, const AbstractMultivector<Derived> &multivector)
    {
        return multivector.derived() * scalar;
    }

    template <class T, class Derived,
              class = typename std::enable_if<TypeTraits<T>::is_scalar_type>::type>  //
    Derived operator/(const T &scalar, const AbstractMultivector<Derived> &multivector)
    {
        return multivector.derived() / scalar;
    }

}  // namespace gafro