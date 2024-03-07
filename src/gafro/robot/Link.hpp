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
    class Joint;

    template <class T>
    class Link
    {
      public:
        Link();

        Link(const Link &other) = delete;

        Link(Link &&other);

        Link &operator=(const Link &other) = delete;

        Link &operator=(Link &&other);

        virtual ~Link() = default;

        void setMass(const T &mass);

        void setCenterOfMass(const Translator<T> &center_of_mass);

        void setInertia(const Inertia<T> &inertia);

        void setName(const std::string &name);

        void setParentJoint(const Joint<T> *parent_joint);

        void addChildJoint(const Joint<T> *child_joint);

        void setAxis(const typename Motor<T>::Generator &axis);

        const T &getMass() const;

        const Translator<T> &getCenterOfMass() const;

        const Inertia<T> &getInertia() const;

        const std::string &getName() const;

        const Joint<T> *getParentJoint() const;

        const std::vector<const Joint<T> *> &getChildJoints() const;

        const typename Motor<T>::Generator &getAxis() const;

      private:
        Translator<T> center_of_mass_;

        Inertia<T> inertia_;

        T mass_;

        std::string name_;

        const Joint<T> *parent_joint_ = nullptr;

        std::vector<const Joint<T> *> child_joints_;

        typename Motor<T>::Generator axis_;
    };
}  // namespace gafro