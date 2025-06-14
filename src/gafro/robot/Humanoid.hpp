// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <gafro/robot/Manipulator.hpp>

namespace gafro
{

    template <class T>
    class Humanoid : public System<T>
    {
      public:
        Humanoid() = delete;

        Humanoid(const Humanoid &other) = delete;

        Humanoid(System<T> &&humanoid);

        Humanoid &operator=(System<T> &&humanoid);

        Humanoid(Humanoid &&humanoid);

        Humanoid &operator=(Humanoid &&humanoid);

        virtual ~Humanoid();

        using System<T>::createKinematicChain;

      protected:
      private:
    };

}  // namespace gafro

#include <gafro/robot/Humanoid.hxx>