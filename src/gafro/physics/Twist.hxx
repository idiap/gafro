#pragma once

#include <gafro/physics/Twist.hpp>
#include <gafro/physics/Wrench.hxx>

namespace gafro
{

    template <class T>
    Twist<T>::Twist()
    {}

    template <class T>
    Twist<T>::Twist(const Base &multivector) : Base(multivector)
    {}

    template <class T>
    Twist<T>::Twist(const typename Base::Parameters &multivector) : Base(multivector)
    {}

    template <class T>
    Twist<T>::~Twist()
    {}

    template <class T>
    typename Twist<T>::Base &Twist<T>::multivector()
    {
        return *this;
    }

    template <class T>
    const typename Twist<T>::Base &Twist<T>::multivector() const
    {
        return *this;
    }

    template <class T>
    typename Twist<T>::Angular Twist<T>::getAngular() const
    {
        return Angular({ this->template get<blades::e23>(),  //
                         this->template get<blades::e13>(),  //
                         this->template get<blades::e12>() });
    }

    template <class T>
    typename Twist<T>::Linear Twist<T>::getLinear() const
    {
        return Linear({ this->template get<blades::e1i>(),  //
                        this->template get<blades::e2i>(),  //
                        this->template get<blades::e3i>() });
    }

    template <class T>
    Twist<T> Twist<T>::transform(const Motor<T> &motor) const
    {
        return motor.apply(multivector());
    }

    template <class T>
    Wrench<T> Twist<T>::commute(const Wrench<T> &wrench) const
    {
        return this->commutatorProduct(wrench);
    }

    template <class T>
    template <class E>
    Twist<T> &Twist<T>::operator=(const Expression<E, Base> &expression)
    {
        Base::operator=(expression);

        return *this;
    }

    template <class T>
    Twist<T> &Twist<T>::operator+=(const Twist<T> &twist)
    {
        this->vector() += twist.vector();

        return *this;
    }

}  // namespace gafro