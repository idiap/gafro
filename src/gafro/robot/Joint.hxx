// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Joint.hpp>

namespace gafro
{

    template <class T>
    Joint<T>::Joint(Type type) : type_(type), parent_link_(nullptr), child_link_(nullptr), index_(-1)
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
        index_ = other.index_;

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
    void Joint<T>::setIndex(const int &index)
    {
        index_ = index;
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
    const int &Joint<T>::getIndex() const
    {
        return index_;
    }

    template <class T>
    bool Joint<T>::isActuated() const
    {
        return type_ != Type::FIXED;
    }

}  // namespace gafro