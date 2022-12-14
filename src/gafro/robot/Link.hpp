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

#include <gafro/algebra.hpp>
//
#include <gafro/physics/Inertia.hxx>

namespace gafro
{
    template <class T>
    class Link
    {
      public:
        Link();

        Link(const T &mass, const Translator<T> &center_of_mass, const Inertia<T> &inertia = Inertia<T>::Zero());

        virtual ~Link() = default;

        void setMass(const T &mass);

        void setCenterOfMass(const Translator<T> &center_of_mass);

        void setInertia(const Inertia<T> &inertia);

        const T &getMass() const;

        const Translator<T> &getCenterOfMass() const;

        const Inertia<T> &getInertia() const;

      private:
        T mass_;

        Translator<T> center_of_mass_;

        Inertia<T> inertia_;
    };
}  // namespace gafro