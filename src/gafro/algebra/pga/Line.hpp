#include <gafro/algebra/pga/ProjectiveGeometricAlgebra.hpp>

namespace gafro::pga
{
    template <class T>
    class Line : public Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>
    {
      public:
        using Base = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>;

        using Base::Base;

        Line(const Base &base) : Base(base) {}

      protected:
      private:
    };
}  // namespace gafro::pga