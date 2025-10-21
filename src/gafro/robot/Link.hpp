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
    class Joint;

    class LinkVisual
    {
      public:
        enum class Type
        {
            SPHERE,
            MESH,
            CYLINDER,
            BOX
        };

        LinkVisual(const Type &type, const Motor<double> &transform)
          : type_(type)
          , transform_(transform)
        {}

        virtual ~LinkVisual() = default;

        virtual std::unique_ptr<LinkVisual> copy() const = 0;

        const Type &getType() const
        {
            return type_;
        }

        const Motor<double> &getTransform() const
        {
            return transform_;
        }

        template <class Derived>
        const Derived *cast() const
        {
            return static_cast<const Derived *>(this);
        }

      private:
        const Type type_;

        Motor<double> transform_;
    };

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

        void setVisual(std::unique_ptr<LinkVisual> &&visual);

        bool hasVisual() const;

        const LinkVisual *getVisual() const;

        std::unique_ptr<Link> copy() const;

      private:
        Translator<T> center_of_mass_;

        Inertia<T> inertia_;

        T mass_;

        std::string name_;

        const Joint<T> *parent_joint_ = nullptr;

        std::vector<const Joint<T> *> child_joints_;

        typename Motor<T>::Generator axis_;

      public:
      private:
        std::unique_ptr<LinkVisual> visual_;
    };
}  // namespace gafro