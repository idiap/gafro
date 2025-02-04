#pragma once

#include <gafro/physics/Wrench.hpp>

namespace gafro
{

    template <class T>
    Wrench<T>::Wrench()
    {}

    template <class T>
    Wrench<T>::Wrench(const Base &multivector) : Base(multivector)
    {}

    template <class T>
    Wrench<T>::Wrench(const T &tx, const T &ty, const T &tz, const T &fx, const T &fy, const T &fz) : Base({ fx, fy, tz, fz, ty, tx })
    {}

    template <class T>
    Wrench<T>::~Wrench()
    {}

    template <class T>
    void Wrench<T>::setLinear(const Linear &linear)
    {
        this->template set<blades::e01>(linear.template get<blades::e01>());
        this->template set<blades::e02>(linear.template get<blades::e02>());
        this->template set<blades::e03>(linear.template get<blades::e03>());
    }

    template <class T>
    void Wrench<T>::setAngular(const Angular &angular)
    {
        this->template set<blades::e12>(angular.template get<blades::e12>());
        this->template set<blades::e13>(angular.template get<blades::e13>());
        this->template set<blades::e23>(angular.template get<blades::e23>());
    }

    template <class T>
    typename Wrench<T>::Linear Wrench<T>::getLinear() const
    {
        Linear linear;

        linear.template set<blades::e01>(this->template get<blades::e01>());
        linear.template set<blades::e02>(this->template get<blades::e02>());
        linear.template set<blades::e03>(this->template get<blades::e03>());

        return linear;
    }

    template <class T>
    typename Wrench<T>::Angular Wrench<T>::getAngular() const
    {
        Angular angular;

        angular.template set<blades::e12>(this->template get<blades::e12>());
        angular.template set<blades::e13>(this->template get<blades::e13>());
        angular.template set<blades::e23>(this->template get<blades::e23>());

        return angular;
    }

    template <class T>
    typename Wrench<T>::Base &Wrench<T>::multivector()
    {
        return *this;
    }

    template <class T>
    const typename Wrench<T>::Base &Wrench<T>::multivector() const
    {
        return *this;
    }

    template <class T>
    Wrench<T> Wrench<T>::transform(const Motor<T> &motor) const
    {
        return motor.apply(multivector());
    }

    template <class T>
    template <class E>
    Wrench<T> &Wrench<T>::operator=(const Expression<E, Base> &expression)
    {
        Base::operator=(expression);

        return *this;
    }

    template <class T>
    Wrench<T> &Wrench<T>::operator+=(const Wrench<T> &wrench)
    {
        this->vector() += wrench.vector();

        return *this;
    }

    template <class T>
    Wrench<T> &Wrench<T>::operator-=(const Wrench<T> &wrench)
    {
        this->vector() -= wrench.vector();

        return *this;
    }

    template <class T>
    Wrench<T> operator-(const Wrench<T> &wrench1, const Wrench<T> &wrench2)
    {
        return Wrench<T>(wrench1.vector() - wrench2.vector());
    }

    template <class T>
    Wrench<T> Wrench<T>::Zero()
    {
        return Wrench<T>(TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                         TypeTraits<T>::Zero());
    }

}  // namespace gafro