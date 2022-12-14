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

#include <gafro/algebra.hpp>

namespace gafro
{

    template <class T>
    class Inertia
    {
      public:
        using Tensor = Eigen::Matrix<T, 3, 3>;

        Inertia() = delete;

        Inertia(const Tensor &tensor);

        virtual ~Inertia() = default;

        typename Rotor<T>::Generator operator()(const typename Rotor<T>::Generator &bivector) const;

        Multivector<T, blades::e1, blades::e2, blades::e3> operator()(const Multivector<T, blades::e1, blades::e2, blades::e3> &vector) const;

        Tensor getRotatedTensor(const Tensor &rotation_matrix) const;

        const Tensor &getTensor() const;

      protected:
      private:
        Tensor tensor_;

      public:
        static Inertia Zero();
    };
}  // namespace gafro