#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Translator Generator creation", "[TranslatorGenerator]" )
{
    Translator<double>::Generator generator;

    REQUIRE( generator.x() == Approx(0.0) );
    REQUIRE( generator.y() == Approx(0.0) );
    REQUIRE( generator.z() == Approx(0.0) );
}


TEST_CASE( "Translator Generator creation from multivector", "[TranslatorGenerator]" )
{
    Multivector<double, blades::e1i, blades::e2i, blades::e3i> mv({ 1.0, 2.0, 3.0 });

    Translator<double>::Generator generator(mv);

    REQUIRE( generator.x() == Approx(mv.get<blades::e1i>()) );
    REQUIRE( generator.y() == Approx(mv.get<blades::e2i>()) );
    REQUIRE( generator.z() == Approx(mv.get<blades::e3i>()) );
}


TEST_CASE( "Translator Generator creation from generator", "[TranslatorGenerator]" )
{
    Translator<double>::Generator generator1({ 1.0, 2.0, 3.0 });
    Translator<double>::Generator generator2(generator1);

    REQUIRE( generator2.x() == Approx(generator1.x()) );
    REQUIRE( generator2.y() == Approx(generator1.y()) );
    REQUIRE( generator2.z() == Approx(generator1.z()) );
}


TEST_CASE( "Translator Generator creation from parameters", "[TranslatorGenerator]" )
{
    Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });

    REQUIRE( generator.x() == Approx(1.0) );
    REQUIRE( generator.y() == Approx(2.0) );
    REQUIRE( generator.z() == Approx(3.0) );

    REQUIRE( generator.x() == Approx(generator.get<blades::e1i>()) );
    REQUIRE( generator.y() == Approx(generator.get<blades::e2i>()) );
    REQUIRE( generator.z() == Approx(generator.get<blades::e3i>()) );
}
