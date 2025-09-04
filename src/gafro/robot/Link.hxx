// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Link.hpp>

namespace gafro
{

    template <class T>
    Link<T>::Link()
      : mass_(TypeTraits<T>::Zero())
    {}

    template <class T>
    Link<T>::Link(Link &&other)
    {
        *this = std::move(other);
    }

    template <class T>
    Link<T> &Link<T>::operator=(Link &&other)
    {
        mass_           = std::move(other.mass_);
        center_of_mass_ = std::move(other.center_of_mass_);
        inertia_        = std::move(other.inertia_);
        name_           = std::move(other.name_);
        parent_joint_   = std::move(other.parent_joint_);
        child_joints_   = std::move(other.child_joints_);
        axis_           = std::move(other.axis_);
        visual_         = std::move(other.visual_);

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

    template <class T>
    void Link<T>::setVisual(std::unique_ptr<LinkVisual> &&visual)
    {
        visual_ = std::move(visual);
    }

    template <class T>
    bool Link<T>::hasVisual() const
    {
        return visual_ != nullptr;
    }

    template <class T>
    const LinkVisual *Link<T>::getVisual() const
    {
        return visual_.get();
    }

    template <class T>
    std::unique_ptr<Link<T>> Link<T>::copy() const
    {
        std::unique_ptr<Link<T>> link = std::make_unique<Link<T>>();

        link->setName(this->getName());
        link->setCenterOfMass(this->getCenterOfMass());
        link->setInertia(this->getInertia());
        link->setMass(this->getMass());
        link->setAxis(this->getAxis());
        if (this->hasVisual())
        {
            link->setVisual(this->getVisual()->copy());
        }

        return link;
    }

}  // namespace gafro