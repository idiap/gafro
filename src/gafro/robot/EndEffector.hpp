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
    class Motor;

    template <class T>
    class EndEffector
    {
      public:
        EndEffector() = default;

        virtual ~EndEffector() = default;

        virtual Motor<T> addTransform(const Motor<T> &base) const = 0;

        virtual std::vector<Point<T>> getGravityContributions(const Motor<T> &ee_motor) const = 0;

        virtual std::vector<T> getMasses() const = 0;

        template <int dof>
        void addGravity(MultivectorMatrix<typename Scalar<T>::Base, dof, 1> &gravity_vector, const Motor<T> &ee_motor,
                        const std::vector<typename Motor<T>::Generator> &axes)
        {
            std::vector<Point<T>> coms = getGravityContributions(ee_motor);
            std::vector<T> masses = getMasses();

            for (unsigned i = 0; i < dof; ++i)
            {
                for (unsigned j = 0; j < coms.size(); ++j)
                {
                    gravity_vector[i] = gravity_vector[i] + Scalar<T>((coms[j] | axes[i]) * Scalar<T>(masses[j]) * E3<T>(T(9.81)));
                }
            }
        }

        template <int dof>
        void addInertia(MultivectorMatrix<Scalar<T>, dof, dof> &inertia_matrix, const std::array<Motor<T>, dof> &joint_motors,
                        const MultivectorMatrix<Motor<T>, dof, dof> &jacobian) const
        {
            // for (unsigned i = 0; i < dof; ++i)
            // {
            //     Rotor<T> rotor = joint_motors[i].getRotor();

            //     MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac;
            //     MultivectorMatrix<typename Rotor<T>::Generator::Base, 1, dof> bjac_inertia;

            //     for (unsigned j = 0; j < dof; ++j)
            //     {
            //         typename Rotor<T>::Generator b = jacobian.coeff(i, j).getRotor().log();

            //         b = Rotor<T>(rotor.reverse()).apply(b).evaluate();

            //         bjac.coeffRef(0, j) = b;
            //         bjac_inertia.coeffRef(0, j) = links_[i].getInertia()(b);
            //     }

            //     inertia_matrix += MultivectorMatrix<typename Rotor<T>::Base, dof, dof>(bjac.transpose() * bjac_inertia).template
            //     extract<Scalar<T>>();
            // }
        }

      protected:
      private:
    };

}  // namespace gafro