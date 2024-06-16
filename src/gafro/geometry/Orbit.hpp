#pragma once

#include <gafro/algebra/cga/ConformalGeometricAlgebra.hpp>
#include <gafro/algebra/cga/Point.hpp>
#include <gafro/geometry/EuclideanPointcloud.hpp>

namespace gafro::geometry
{

    template <class T, int... blades>                                                       //
        requires((ConformalGeometricAlgebra::BladeBitmap::getGrade<blades>() == 2) && ...)  //
    class Orbit
    {
      public:
        using Generator = Multivector<T, blades...>;

      public:
        Orbit();

        Orbit(const Generator &generator);

        virtual ~Orbit();

        void setGenerator(const Generator &generator);

        const Generator &getGenerator() const;

        EuclideanPointcloud<T> createEuclideanPointcloud(const Point<T> &point) const;

      protected:
      private:
        Generator generator_;
    };

}  // namespace gafro::geometry

#include <gafro/geometry/Orbit.hxx>