#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Translator creation", "[Translator]" )
{
    Translator<double> translator;

    REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
    REQUIRE( translator.get<blades::e1i>() == Approx(0.0) );
    REQUIRE( translator.get<blades::e2i>() == Approx(0.0) );
    REQUIRE( translator.get<blades::e3i>() == Approx(0.0) );
}


TEST_CASE( "Translator creation from generator", "[Translator]" )
{
    Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
    Translator<double> translator(generator);

    REQUIRE( translator.get<blades::scalar>() == Approx(1.0) );
    REQUIRE( translator.get<blades::e1i>() == Approx(-0.5) );
    REQUIRE( translator.get<blades::e2i>() == Approx(-1.0) );
    REQUIRE( translator.get<blades::e3i>() == Approx(-1.5) );
}


TEST_CASE( "Translator get log", "[Translator]" )
{
    Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
    Translator<double> translator(generator);

    Translator<double>::Generator log = translator.log();

    REQUIRE( log.x() == Approx(1.0) );
    REQUIRE( log.y() == Approx(2.0) );
    REQUIRE( log.z() == Approx(3.0) );
}
