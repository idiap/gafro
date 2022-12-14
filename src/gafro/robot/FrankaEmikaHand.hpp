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

#include <gafro/robot/EndEffector.hpp>
#include <gafro/robot/Link.hpp>
#include <gafro/robot/PrismaticJoint.hpp>

namespace gafro
{

    template <class T>
    class FrankaEmikaHand : public EndEffector<T>
    {
      public:
        FrankaEmikaHand()
          : left_finger_joint_({ 0.0, 0.0, 0.0584, 0.0, 0.0, 0.0 }, 2),  //
            right_finger_joint_({ 0.0, 0.0, 0.0584, 0.0, 0.0, M_PI }, -2)
        {
            base_link_.setMass(0.73);
            base_link_.setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ -0.01, 0.0, 0.03 })));
            base_link_.setInertia((Eigen::Matrix<T, 3, 3>() << 0.001, 0.0, 0.0,  //
                                   0.0, 0.0025, 0.0,                             //
                                   0.0, 0.0, 0.0017)
                                    .finished());

            left_finger_link_.setMass(0.015);
            left_finger_link_.setCenterOfMass(Translator<T>());
            left_finger_link_.setInertia((Eigen::Matrix<T, 3, 3>() << 2.3749999999999997e-06, 0.0, 0.0,  //
                                          0.0, 2.3749999999999997e-06, 0.0,                              //
                                          0.0, 0.0, 7.5e-07)
                                           .finished());

            right_finger_link_.setMass(0.015);
            right_finger_link_.setCenterOfMass(Translator<T>());
            right_finger_link_.setInertia((Eigen::Matrix<T, 3, 3>() << 2.3749999999999997e-06, 0.0, 0.0,  //
                                           0.0, 2.3749999999999997e-06, 0.0,                              //
                                           0.0, 0.0, 7.5e-07)
                                            .finished());

            tip_frame_ = Translator<T>(typename Translator<T>::Generator({ 0.0, 0.0, 0.1034 }));
        }

        virtual ~FrankaEmikaHand() = default;

        Motor<T> addTransform(const Motor<T> &base) const
        {
            return base * tip_frame_;
        }

        std::vector<Point<T>> getGravityContributions(const Motor<T> &ee_motor) const
        {
            std::vector<Point<T>> coms;

            coms.push_back(Motor<T>(ee_motor * base_link_.getCenterOfMass()).apply(Point<T>()));
            coms.push_back(Motor<T>(ee_motor * left_finger_joint_.getMotor(0.0) * left_finger_link_.getCenterOfMass()).apply(Point<T>()));
            coms.push_back(Motor<T>(ee_motor * right_finger_joint_.getMotor(0.0) * right_finger_link_.getCenterOfMass()).apply(Point<T>()));

            return coms;
        }

        std::vector<T> getMasses() const
        {
            return { base_link_.getMass(), left_finger_link_.getMass(), right_finger_link_.getMass() };
        }

        const Link<T> &getBaseLink() const
        {
            return base_link_;
        }

        const Link<T> &getLeftFingerLink() const
        {
            return left_finger_link_;
        }

        const Link<T> &getRightFingerLink() const
        {
            return right_finger_link_;
        }

        const PrismaticJoint<T> &getLeftFingerJoint() const
        {
            return left_finger_joint_;
        }

        const PrismaticJoint<T> &getRightFingerJoint() const
        {
            return right_finger_joint_;
        }

      protected:
      private:
        Link<T> base_link_;

        Link<T> left_finger_link_;

        Link<T> right_finger_link_;

        PrismaticJoint<T> left_finger_joint_;

        PrismaticJoint<T> right_finger_joint_;

        Translator<T> tip_frame_;
    };

}  // namespace gafro