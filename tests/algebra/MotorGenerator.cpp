#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Motor Generator creation", "[MotorGenerator]" )
{
    Motor<double>::Generator generator;

    Rotor<double>::Generator rotorGenerator = generator.getRotorGenerator();
    Translator<double>::Generator translatorGenerator = generator.getTranslatorGenerator();

    REQUIRE( rotorGenerator.e23() == Approx(0.0) );
    REQUIRE( rotorGenerator.e13() == Approx(0.0) );
    REQUIRE( rotorGenerator.e12() == Approx(0.0) );

    REQUIRE( translatorGenerator.x() == Approx(0.0) );
    REQUIRE( translatorGenerator.y() == Approx(0.0) );
    REQUIRE( translatorGenerator.z() == Approx(0.0) );
}


TEST_CASE( "Motor Generator creation from multivector", "[MotorGenerator]" )
{
    Multivector<double, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    Motor<double>::Generator generator(mv);

    Rotor<double>::Generator rotorGenerator = generator.getRotorGenerator();
    Translator<double>::Generator translatorGenerator = generator.getTranslatorGenerator();

    REQUIRE( rotorGenerator.e23() == Approx(mv.get<blades::e23>()) );
    REQUIRE( rotorGenerator.e13() == Approx(mv.get<blades::e13>()) );
    REQUIRE( rotorGenerator.e12() == Approx(mv.get<blades::e12>()) );

    REQUIRE( translatorGenerator.x() == Approx(mv.get<blades::e1i>()) );
    REQUIRE( translatorGenerator.y() == Approx(mv.get<blades::e2i>()) );
    REQUIRE( translatorGenerator.z() == Approx(mv.get<blades::e3i>()) );
}


TEST_CASE( "Motor Generator creation from generator", "[MotorGenerator]" )
{
    Motor<double>::Generator generator1({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Motor<double>::Generator generator2(generator1);

    Rotor<double>::Generator rotorGenerator1 = generator1.getRotorGenerator();
    Translator<double>::Generator translatorGenerator1 = generator1.getTranslatorGenerator();

    Rotor<double>::Generator rotorGenerator2 = generator2.getRotorGenerator();
    Translator<double>::Generator translatorGenerator2 = generator2.getTranslatorGenerator();

    REQUIRE( rotorGenerator2.e23() == Approx(rotorGenerator1.e23()) );
    REQUIRE( rotorGenerator2.e13() == Approx(rotorGenerator1.e13()) );
    REQUIRE( rotorGenerator2.e12() == Approx(rotorGenerator1.e12()) );

    REQUIRE( translatorGenerator2.x() == Approx(translatorGenerator1.x()) );
    REQUIRE( translatorGenerator2.y() == Approx(translatorGenerator1.y()) );
    REQUIRE( translatorGenerator2.z() == Approx(translatorGenerator1.z()) );
}


TEST_CASE( "Motor Generator creation from parameters", "[MotorGenerator]" )
{
    Motor<double>::Generator generator({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    Rotor<double>::Generator rotorGenerator = generator.getRotorGenerator();
    Translator<double>::Generator translatorGenerator = generator.getTranslatorGenerator();

    REQUIRE( rotorGenerator.e23() == Approx(1.0) );
    REQUIRE( rotorGenerator.e13() == Approx(2.0) );
    REQUIRE( rotorGenerator.e12() == Approx(3.0) );

    REQUIRE( translatorGenerator.x() == Approx(4.0) );
    REQUIRE( translatorGenerator.y() == Approx(5.0) );
    REQUIRE( translatorGenerator.z() == Approx(6.0) );
}


TEST_CASE( "Motor Generator creation from matrices", "[MotorGenerator]" )
{
    Eigen::Matrix<double, 3, 1> p1({ 1.0, 2.0, 3.0 });
    Eigen::Matrix<double, 3, 1> p2({ 4.0, 5.0, 6.0 });

    Motor<double>::Generator generator(p1, p2);

    Rotor<double>::Generator rotorGenerator = generator.getRotorGenerator();
    Translator<double>::Generator translatorGenerator = generator.getTranslatorGenerator();

    REQUIRE( rotorGenerator.e23() == Approx(1.0) );
    REQUIRE( rotorGenerator.e13() == Approx(2.0) );
    REQUIRE( rotorGenerator.e12() == Approx(3.0) );

    REQUIRE( translatorGenerator.x() == Approx(4.0) );
    REQUIRE( translatorGenerator.y() == Approx(5.0) );
    REQUIRE( translatorGenerator.z() == Approx(6.0) );
}
