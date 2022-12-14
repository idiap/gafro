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

#include <gafro/robot/Manipulator.hpp>
//
#include <gafro/robot/FrankaEmikaHand.hpp>

namespace gafro
{

    template <class T>
    class FrankaEmikaRobot : public Manipulator<T, 7>
    {
      public:
        FrankaEmikaRobot();

        virtual ~FrankaEmikaRobot();

        const FrankaEmikaHand<T> *getFrankaHand() const;

      protected:
      private:
    };

    template <class T>
    FrankaEmikaRobot<T>::FrankaEmikaRobot()  //
      : Manipulator<T, 7>(
          {
            std::array<T, 3>({ 0.0, 0.333, 0.0 }),              //
            std::array<T, 3>({ 0.0, 0.0, -M_PI / 2.0 }),        //
            std::array<T, 3>({ 0.0, 0.316, M_PI / 2.0 }),       //
            std::array<T, 3>({ 0.0825, 0.0, M_PI / 2.0 }),      //
            std::array<T, 3>({ -0.0825, 0.384, -M_PI / 2.0 }),  //
            std::array<T, 3>({ 0.0, 0.0, M_PI / 2.0 }),         //
            std::array<T, 3>({ 0.088, 0.0, M_PI / 2.0 })        //
          },
          { 0.0, 0.107, -M_PI / 4.0 },  //
          std::make_unique<FrankaEmikaHand<T>>())
    {
        this->setJointLimits({ -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 },
                             { 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973 });

        this->getLink(0).setMass(4.970684);
        this->getLink(0).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ 0.003875, 0.002081, -0.04762 })));
        this->getLink(0).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.70337, -0.000139, 0.006772, -0.000139, 0.70661, 0.019169, 0.006772, 0.019169, 0.009117).finished());

        this->getLink(1).setMass(0.646926);
        this->getLink(1).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ -0.003141, -0.02872, 0.003495 })));
        this->getLink(1).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.007962, -0.003925, 0.010254, -0.003925, 0.02811, 0.000704, 0.010254, 0.000704, 0.025995).finished());

        this->getLink(2).setMass(3.228604);
        this->getLink(2).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ 0.027518, 0.039252, -0.066502 })));
        this->getLink(2).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.037242, -0.004761, -0.011396, -0.004761, 0.036155, -0.012805, -0.011396, -0.012805, 0.01083).finished());

        this->getLink(3).setMass(3.587895);
        this->getLink(3).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ -0.05317, 0.104419, 0.027454 })));
        this->getLink(3).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.025853, 0.007796, -0.001332, 0.007796, 0.019552, 0.008641, -0.001332, 0.008641, 0.028323).finished());

        this->getLink(4).setMass(1.225946);
        this->getLink(4).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ -0.011953, 0.041065, -0.038437 })));
        this->getLink(4).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.035549, -0.002117, -0.004037, -0.002117, 0.029474, 0.000229, -0.004037, 0.000229, 0.008627).finished());

        this->getLink(5).setMass(1.666555);
        this->getLink(5).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ 0.060149, -0.014117, -0.010517 })));
        this->getLink(5).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.001964, 0.000109, -0.001158, 0.000109, 0.004354, 0.000341, -0.001158, 0.000341, 0.005433).finished());

        this->getLink(6).setMass(7.35522e-01);
        this->getLink(6).setCenterOfMass(Translator<T>(typename Translator<T>::Generator({ 0.010517, -0.004252, 0.061597 })));
        this->getLink(6).setInertia(
          (Eigen::Matrix<T, 3, 3>() << 0.012516, -0.000428, -0.001196, -0.000428, 0.010027, -0.000741, -0.001196, -0.000741, 0.004815).finished());
    }

    template <class T>
    FrankaEmikaRobot<T>::~FrankaEmikaRobot()
    {}

    template <class T>
    const FrankaEmikaHand<T> *FrankaEmikaRobot<T>::getFrankaHand() const
    {
        return static_cast<const FrankaEmikaHand<T> *>(this->getEndEffector());
    }

}  // namespace gafro