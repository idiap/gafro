// SPDX-FileCopyrightText: Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: MPL-2.0

#pragma once

namespace gafro
{
    template <class Derived>
    class AbstractExpression
    {
      public:
      public:
        AbstractExpression();

        virtual ~AbstractExpression();

        Derived &derived();

        const Derived &derived() const;

      protected:
      private:
    };

}  // namespace gafro
