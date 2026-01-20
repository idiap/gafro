#include <gafro/algebra/pga/ProjectiveGeometricAlgebra.hpp>
//
#include <gafro/algebra/cga/Line.hpp>

namespace gafro::pga
{
    template <class T>
    class Line : public Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>
    {
      public:
        using Base = Multivector<T, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23>;

        using Base::Base;

        Line(const Base &base)
          : Base(base)
        {}

        Line(const gafro::Line<T> &cga_line)
        {
            this->template set<blades::e12>(-cga_line.template get<gafro::blades::e03i>());
            this->template set<blades::e13>(cga_line.template get<gafro::blades::e02i>());
            this->template set<blades::e23>(-cga_line.template get<gafro::blades::e01i>());
            this->template set<blades::e01>(-cga_line.template get<gafro::blades::e23i>());
            this->template set<blades::e02>(cga_line.template get<gafro::blades::e13i>());
            this->template set<blades::e03>(-cga_line.template get<gafro::blades::e12i>());
        }

      protected:
      private:
    };
}  // namespace gafro::pga