// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Link;

    template <class T>
    class Joint
    {
      public:
        enum class Type
        {
            FIXED,
            REVOLUTE,
            PRISMATIC
        };

        Joint(Type type);

        Joint(const Joint &other) = delete;

        Joint(Joint &&other);

        Joint &operator=(const Joint &other) = delete;

        Joint &operator=(Joint &&other);

        virtual ~Joint();

        struct Limits
        {
            template <class S>
            typename Joint<S>::Limits cast() const
            {
                return { TypeTraits<S>::Value(position_lower),  //
                         TypeTraits<S>::Value(position_upper),  //
                         TypeTraits<S>::Value(velocity),        //
                         TypeTraits<S>::Value(torque) };
            }

            T position_lower;

            T position_upper;

            T velocity;

            T torque;
        };

        // setter functions
        void setFrame(const Motor<T> &frame);

        void setLimits(const Limits &limits);

        void setName(const std::string &name);

        void setParentLink(const Link<T> *parent_link);

        void setChildLink(const Link<T> *child_link);

        void setIndex(const int &index);

        // getter functions
        const std::string &getName() const;

        const Motor<T> &getFrame() const;

        const Type &getType() const;

        const Limits &getLimits() const;

        const Link<T> *getParentLink() const;

        const Link<T> *getChildLink() const;

        const int &getIndex() const;

        //

        bool isActuated() const;

        // virtual functions
        virtual Motor<T> getMotor(const T &angle) const = 0;

        virtual Motor<T> getMotorDerivative(const T &angle) const = 0;

        virtual typename Motor<T>::Generator getCurrentAxis(const Motor<T> &motor) const = 0;

      protected:
      private:
        Type type_;

        std::string name_;

        Motor<T> frame_;

        Limits limits_;

        const Link<T> *parent_link_ = nullptr;

        const Link<T> *child_link_ = nullptr;

        int index_;
    };

}  // namespace gafro