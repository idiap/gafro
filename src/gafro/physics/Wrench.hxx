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
    Wrench<T>::Wrench(const Parameters &multivector) : Base(multivector)
    {}

    template <class T>
    Wrench<T>::~Wrench()
    {}

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
        return Wrench<T>({ TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(), TypeTraits<T>::Zero(),
                           TypeTraits<T>::Zero() });
    }

}  // namespace gafro