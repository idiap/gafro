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

#include <gafro/algebra/Multivector.hxx>
#include <gafro/algebra/MultivectorMatrix.hpp>
//
#include <gafro/algebra/AbstractMultivector.hxx>
//
#include <gafro/algebra/AbstractExpression.hxx>
#include <gafro/algebra/CommutatorProduct.hpp>
#include <gafro/algebra/GeometricProduct.hpp>
#include <gafro/algebra/Grade.hpp>
#include <gafro/algebra/InnerProduct.hpp>
#include <gafro/algebra/OuterProduct.hpp>
#include <gafro/algebra/Reflection.hpp>
#include <gafro/algebra/Reverse.hpp>
#include <gafro/algebra/SandwichProduct.hpp>
#include <gafro/algebra/Sum.hpp>
//
#include <gafro/algebra/Operators.hxx>
//
#include <gafro/algebra/cga/Blades.hpp>
#include <gafro/algebra/cga/Circle.hxx>
#include <gafro/algebra/cga/DirectionVector.hxx>
#include <gafro/algebra/cga/Line.hxx>
#include <gafro/algebra/cga/Plane.hxx>
#include <gafro/algebra/cga/Point.hxx>
#include <gafro/algebra/cga/PointPair.hxx>
#include <gafro/algebra/cga/Sphere.hxx>
#include <gafro/algebra/cga/TangentVector.hxx>
#include <gafro/algebra/cga/Vector.hxx>
//
#include <gafro/algebra/cga/Dilator.hpp>
#include <gafro/algebra/cga/Motor.hxx>
#include <gafro/algebra/cga/MotorGenerator.hxx>
#include <gafro/algebra/cga/Rotor.hxx>
#include <gafro/algebra/cga/RotorGenerator.hxx>
#include <gafro/algebra/cga/Translator.hxx>
#include <gafro/algebra/cga/TranslatorGenerator.hxx>