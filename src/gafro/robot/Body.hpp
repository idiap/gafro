// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>
//
#include <gafro/physics/Inertia.hxx>

namespace gafro
{
    template <class T>
    class Body
    {
      public:
        Body();

        Body(const Body &other) = delete;

        Body(Body &&other);

        virtual ~Body() = default;

        void setCenterOfMass(const Translator<T> &center_of_mass);

        void setInertia(const Inertia<T> &inertia);

        void setAxis(const typename Motor<T>::Generator &axis);

        void setMass(const T &mass);

        const Translator<T> &getCenterOfMass() const;

        const Inertia<T> &getInertia() const;

        const typename Motor<T>::Generator &getAxis() const;

        const T &getMass() const;

      private:
        Translator<T> center_of_mass_;

        Inertia<T> inertia_;

        typename Motor<T>::Generator axis_;

        T mass_;
    };
}  // namespace gafro
