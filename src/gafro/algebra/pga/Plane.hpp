#include <gafro/algebra/pga/ProjectiveGeometricAlgebra.hpp>
//
#include <gafro/algebra/cga/Plane.hpp>

namespace gafro::pga
{
    template <class T>
    class Plane : public Multivector<T, blades::e0, blades::e1, blades::e2, blades::e3>
    {
      public:
        using Base = Multivector<T, blades::e0, blades::e1, blades::e2, blades::e3>;

        using Base::Base;

        Plane(const T &x, const T &y, const T &z, const T &d) : Base({ d, x, y, z }) {}

      protected:
      private:
    };
}  // namespace gafro::pga