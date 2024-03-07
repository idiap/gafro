#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Rotor Generator creation", "[RotorGenerator]" )
{
    Rotor<double>::Generator generator;

    REQUIRE( generator.e23() == Approx(0.0) );
    REQUIRE( generator.e13() == Approx(0.0) );
    REQUIRE( generator.e12() == Approx(0.0) );
}


TEST_CASE( "Rotor Generator creation from multivector", "[RotorGenerator]" )
{
    Multivector<double, blades::e23, blades::e13, blades::e12> mv({ 1.0, 2.0, 3.0 });

    Rotor<double>::Generator generator(mv);

    REQUIRE( generator.e23() == Approx(mv.get<blades::e23>()) );
    REQUIRE( generator.e13() == Approx(mv.get<blades::e13>()) );
    REQUIRE( generator.e12() == Approx(mv.get<blades::e12>()) );
}


TEST_CASE( "Rotor Generator creation from generator", "[RotorGenerator]" )
{
    Rotor<double>::Generator generator1({ 1.0, 2.0, 3.0 });
    Rotor<double>::Generator generator2(generator1);

    REQUIRE( generator2.e23() == Approx(generator1.e23()) );
    REQUIRE( generator2.e13() == Approx(generator1.e13()) );
    REQUIRE( generator2.e12() == Approx(generator1.e12()) );
}


TEST_CASE( "Rotor Generator creation from parameters", "[RotorGenerator]" )
{
    Rotor<double>::Generator generator({ 1.0, 2.0, 3.0 });

    REQUIRE( generator.e23() == Approx(1.0) );
    REQUIRE( generator.e13() == Approx(2.0) );
    REQUIRE( generator.e12() == Approx(3.0) );

    REQUIRE( generator.e23() == Approx(generator.get<blades::e23>()) );
    REQUIRE( generator.e13() == Approx(generator.get<blades::e13>()) );
    REQUIRE( generator.e12() == Approx(generator.get<blades::e12>()) );
}


TEST_CASE( "Rotor Generator creation from expression", "[RotorGenerator]" )
{

    Multivector<double, blades::e23, blades::e13, blades::e12> mv1({ 1.0, 2.0, 3.0 });
    Multivector<double, blades::e23, blades::e13, blades::e12> mv2({ 10.0, 20.0, 30.0 });

    Sum<Rotor<double>::Generator::Base, Rotor<double>::Generator::Base, detail::AdditionOperator> sum(mv1, mv2);

    Rotor<double>::Generator generator(sum);

    REQUIRE( generator.e23() == Approx(11.0) );
    REQUIRE( generator.e13() == Approx(22.0) );
    REQUIRE( generator.e12() == Approx(33.0) );
}


TEST_CASE( "Rotor Generator assignation from expression", "[RotorGenerator]" )
{

    Multivector<double, blades::e23, blades::e13, blades::e12> mv1({ 1.0, 2.0, 3.0 });
    Multivector<double, blades::e23, blades::e13, blades::e12> mv2({ 10.0, 20.0, 30.0 });

    Sum<Rotor<double>::Generator::Base, Rotor<double>::Generator::Base, detail::AdditionOperator> sum(mv1, mv2);

    Rotor<double>::Generator generator = sum;

    REQUIRE( generator.e23() == Approx(11.0) );
    REQUIRE( generator.e13() == Approx(22.0) );
    REQUIRE( generator.e12() == Approx(33.0) );
}
