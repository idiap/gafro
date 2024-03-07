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

#include <gafro/robot/Joint.hpp>

namespace gafro
{

    template <class T>
    Joint<T>::Joint(Type type) : type_(type), parent_link_(nullptr), child_link_(nullptr)
    {}

    template <class T>
    Joint<T>::Joint(Joint &&other)
    {
        *this = std::move(other);
    }

    template <class T>
    Joint<T> &Joint<T>::operator=(Joint &&other)
    {
        type_ = std::move(other.type_);
        name_ = std::move(other.name_);
        frame_ = std::move(other.frame_);
        limits_ = std::move(other.limits_);
        parent_link_ = std::move(other.parent_link_);
        child_link_ = std::move(other.child_link_);

        return *this;
    }

    template <class T>
    Joint<T>::~Joint()
    {}

    template <class T>
    void Joint<T>::setFrame(const Motor<T> &frame)
    {
        frame_ = frame;
    }

    template <class T>
    void Joint<T>::setLimits(const Limits &limits)
    {
        limits_ = limits;
    }

    template <class T>
    void Joint<T>::setName(const std::string &name)
    {
        name_ = name;
    }

    template <class T>
    void Joint<T>::setParentLink(const Link<T> *parent_link)
    {
        parent_link_ = parent_link;
    }

    template <class T>
    void Joint<T>::setChildLink(const Link<T> *child_link)
    {
        child_link_ = child_link;
    }

    template <class T>
    const std::string &Joint<T>::getName() const
    {
        return name_;
    }

    template <class T>
    const Motor<T> &Joint<T>::getFrame() const
    {
        return frame_;
    }

    template <class T>
    const typename Joint<T>::Type &Joint<T>::getType() const
    {
        return type_;
    }

    template <class T>
    const typename Joint<T>::Limits &Joint<T>::getLimits() const
    {
        return limits_;
    }

    template <class T>
    const Link<T> *Joint<T>::getParentLink() const
    {
        return parent_link_;
    }

    template <class T>
    const Link<T> *Joint<T>::getChildLink() const
    {
        return child_link_;
    }

    template <class T>
    bool Joint<T>::isActuated() const
    {
        return type_ != Type::FIXED;
    }

}  // namespace gafro