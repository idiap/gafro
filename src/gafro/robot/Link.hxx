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

#include <gafro/robot/Link.hpp>

namespace gafro
{

    template <class T>
    Link<T>::Link() : mass_(TypeTraits<T>::Zero())
    {}

    template <class T>
    Link<T>::Link(Link &&other)
    {
        *this = std::move(other);
    }

    template <class T>
    Link<T> &Link<T>::operator=(Link &&other)
    {
        mass_ = std::move(other.mass_);
        center_of_mass_ = std::move(other.center_of_mass_);
        inertia_ = std::move(other.inertia_);
        name_ = std::move(other.name_);
        parent_joint_ = std::move(other.parent_joint_);
        child_joints_ = std::move(other.child_joints_);
        axis_ = std::move(other.axis_);

        return *this;
    }

    template <class T>
    void Link<T>::setMass(const T &mass)
    {
        mass_ = mass;
    }

    template <class T>
    void Link<T>::setCenterOfMass(const Translator<T> &center_of_mass)
    {
        center_of_mass_ = center_of_mass;

        if (parent_joint_)
        {
            axis_ = parent_joint_->getCurrentAxis(Translator<T>(center_of_mass_.reverse()));
        }
    }

    template <class T>
    void Link<T>::setInertia(const Inertia<T> &inertia)
    {
        inertia_ = inertia;
    }

    template <class T>
    void Link<T>::setParentJoint(const Joint<T> *parent_joint)
    {
        parent_joint_ = parent_joint;
    }

    template <class T>
    void Link<T>::addChildJoint(const Joint<T> *child_joint)
    {
        child_joints_.push_back(child_joint);
    }

    template <class T>
    void Link<T>::setAxis(const typename Motor<T>::Generator &axis)
    {
        axis_ = axis;
    }

    template <class T>
    const T &Link<T>::getMass() const
    {
        return mass_;
    }

    template <class T>
    const Translator<T> &Link<T>::getCenterOfMass() const
    {
        return center_of_mass_;
    }

    template <class T>
    const Inertia<T> &Link<T>::getInertia() const
    {
        return inertia_;
    }

    template <class T>
    void Link<T>::setName(const std::string &name)
    {
        name_ = name;
    }

    template <class T>
    const std::string &Link<T>::getName() const
    {
        return name_;
    }

    template <class T>
    const Joint<T> *Link<T>::getParentJoint() const
    {
        return parent_joint_;
    }

    template <class T>
    const std::vector<const Joint<T> *> &Link<T>::getChildJoints() const
    {
        return child_joints_;
    }

    template <class T>
    const typename Motor<T>::Generator &Link<T>::getAxis() const
    {
        return axis_;
    }

}  // namespace gafro