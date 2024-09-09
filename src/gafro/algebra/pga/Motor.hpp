#include <gafro/algebra/pga/ProjectiveGeometricAlgebra.hpp>
//
#include <gafro/algebra/cga/Motor.hpp>

namespace gafro::pga
{
    template <class T>
    class Motor : public Multivector<T, blades::scalar, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23, blades::e0123>
    {
      public:
        using Base = Multivector<T, blades::scalar, blades::e01, blades::e02, blades::e12, blades::e03, blades::e13, blades::e23, blades::e0123>;

        Motor() = default;

        Motor(const Base &dq) : Base(dq) {}

        Motor(Base &&dq) : Base(std::move(dq)) {}

        Motor(const typename Base::Parameters &dq) : Base(dq) {}

        Motor(const gafro::Motor<T> &cga_motor)
        {
            this->template set<blades::scalar>(cga_motor.template get<gafro::blades::scalar>());
            this->template set<blades::e23>(-cga_motor.template get<gafro::blades::e23>());
            this->template set<blades::e12>(-cga_motor.template get<gafro::blades::e12>());
            this->template set<blades::e13>(cga_motor.template get<gafro::blades::e13>());
            this->template set<blades::e01>(-cga_motor.template get<gafro::blades::e1i>());
            this->template set<blades::e02>(cga_motor.template get<gafro::blades::e2i>());
            this->template set<blades::e03>(-cga_motor.template get<gafro::blades::e3i>());
            this->template set<blades::e0123>(cga_motor.template get<gafro::blades::e123i>());
        }

        virtual ~Motor() = default;

        operator gafro::Motor<T>() const
        {
            gafro::Motor<T> cga_motor;

            cga_motor.template set<gafro::blades::scalar>(this->template get<blades::scalar>());
            cga_motor.template set<gafro::blades::e23>(-this->template get<blades::e23>());
            cga_motor.template set<gafro::blades::e12>(-this->template get<blades::e12>());
            cga_motor.template set<gafro::blades::e13>(this->template get<blades::e13>());
            cga_motor.template set<gafro::blades::e1i>(-this->template get<blades::e01>());
            cga_motor.template set<gafro::blades::e2i>(this->template get<blades::e02>());
            cga_motor.template set<gafro::blades::e3i>(-this->template get<blades::e03>());
            cga_motor.template set<gafro::blades::e123i>(this->template get<blades::e0123>());

            return cga_motor;
        }

        using Base::vector;

      protected:
      private:
    };
}  // namespace gafro::pga