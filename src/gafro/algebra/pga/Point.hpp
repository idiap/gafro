#include <gafro/algebra/pga/ProjectiveGeometricAlgebra.hpp>

namespace gafro::pga
{
    template <class T>
    class Point : public Multivector<T, blades::e012, blades::e013, blades::e023, blades::e123>
    {
      public:
        using Base = Multivector<T, blades::e012, blades::e013, blades::e023, blades::e123>;

        using Base::Base;

        Point(const T &x, const T &y, const T &z)
        {
            this->template set<blades::e012>(z);
            this->template set<blades::e013>(y);
            this->template set<blades::e023>(x);
            this->template set<blades::e123>(1.0);
        }

      protected:
      private:
    };
}  // namespace gafro::pga