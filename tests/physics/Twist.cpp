#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Twist creation", "[Twist]" )
{
    Twist<double> twist;

    REQUIRE( twist.get<blades::e23>() == Approx(0.0) );
    REQUIRE( twist.get<blades::e13>() == Approx(0.0) );
    REQUIRE( twist.get<blades::e12>() == Approx(0.0) );
    REQUIRE( twist.get<blades::e1i>() == Approx(0.0) );
    REQUIRE( twist.get<blades::e2i>() == Approx(0.0) );
    REQUIRE( twist.get<blades::e3i>() == Approx(0.0) );

    SECTION( "multivector" )
    {
        auto mv = twist.multivector();

        REQUIRE( mv.size == twist.size );

        REQUIRE( mv.get<blades::e23>() == Approx(0.0) );
        REQUIRE( mv.get<blades::e13>() == Approx(0.0) );
        REQUIRE( mv.get<blades::e12>() == Approx(0.0) );
        REQUIRE( mv.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( mv.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( mv.get<blades::e3i>() == Approx(0.0) );
    }

    SECTION( "angular" )
    {
        auto angular = twist.getAngular();

        REQUIRE( angular.size == 3 );

        REQUIRE( angular.get<blades::e23>() == Approx(0.0) );
        REQUIRE( angular.get<blades::e13>() == Approx(0.0) );
        REQUIRE( angular.get<blades::e12>() == Approx(0.0) );
    }

    SECTION( "linear" )
    {
        auto linear = twist.getLinear();

        REQUIRE( linear.size == 3 );

        REQUIRE( linear.get<blades::e1i>() == Approx(0.0) );
        REQUIRE( linear.get<blades::e2i>() == Approx(0.0) );
        REQUIRE( linear.get<blades::e3i>() == Approx(0.0) );
    }
}


TEST_CASE( "Twist creation from parameters", "[Twist]" )
{
    Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    REQUIRE( twist.get<blades::e23>() == Approx(1.0) );
    REQUIRE( twist.get<blades::e13>() == Approx(2.0) );
    REQUIRE( twist.get<blades::e12>() == Approx(3.0) );
    REQUIRE( twist.get<blades::e1i>() == Approx(4.0) );
    REQUIRE( twist.get<blades::e2i>() == Approx(5.0) );
    REQUIRE( twist.get<blades::e3i>() == Approx(6.0) );

    SECTION( "multivector" )
    {
        auto mv = twist.multivector();

        REQUIRE( mv.bits().size() == twist.bits().size() );

        REQUIRE( mv.get<blades::e23>() == Approx(1.0) );
        REQUIRE( mv.get<blades::e13>() == Approx(2.0) );
        REQUIRE( mv.get<blades::e12>() == Approx(3.0) );
        REQUIRE( mv.get<blades::e1i>() == Approx(4.0) );
        REQUIRE( mv.get<blades::e2i>() == Approx(5.0) );
        REQUIRE( mv.get<blades::e3i>() == Approx(6.0) );
    }

    SECTION( "angular" )
    {
        auto angular = twist.getAngular();

        REQUIRE( angular.size == 3 );

        REQUIRE( angular.get<blades::e23>() == Approx(1.0) );
        REQUIRE( angular.get<blades::e13>() == Approx(2.0) );
        REQUIRE( angular.get<blades::e12>() == Approx(3.0) );
    }

    SECTION( "linear" )
    {
        auto linear = twist.getLinear();

        REQUIRE( linear.size == 3 );

        REQUIRE( linear.get<blades::e1i>() == Approx(4.0) );
        REQUIRE( linear.get<blades::e2i>() == Approx(5.0) );
        REQUIRE( linear.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Twist creation from twist", "[Twist]" )
{
    Twist<double> twist1({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Twist<double> twist2(twist1);

    REQUIRE( twist2.get<blades::e23>() == Approx(1.0) );
    REQUIRE( twist2.get<blades::e13>() == Approx(2.0) );
    REQUIRE( twist2.get<blades::e12>() == Approx(3.0) );
    REQUIRE( twist2.get<blades::e1i>() == Approx(4.0) );
    REQUIRE( twist2.get<blades::e2i>() == Approx(5.0) );
    REQUIRE( twist2.get<blades::e3i>() == Approx(6.0) );
}


TEST_CASE( "Twist creation from Multivector", "[Twist]" )
{
    Multivector<double, blades::e23, blades::e13, blades::e12, blades::e1i, blades::e2i, blades::e3i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Twist<double> twist(mv);

    REQUIRE( twist.get<blades::e23>() == Approx(1.0) );
    REQUIRE( twist.get<blades::e13>() == Approx(2.0) );
    REQUIRE( twist.get<blades::e12>() == Approx(3.0) );
    REQUIRE( twist.get<blades::e1i>() == Approx(4.0) );
    REQUIRE( twist.get<blades::e2i>() == Approx(5.0) );
    REQUIRE( twist.get<blades::e3i>() == Approx(6.0) );
}


TEST_CASE( "Twist transform by motor", "[Twist]" )
{
    Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    SECTION( "default" )
    {
        Motor<double> motor;

        Twist<double> twist2 = twist.transform(motor);

        REQUIRE( twist2.get<blades::e23>() == Approx(1.0) );
        REQUIRE( twist2.get<blades::e13>() == Approx(2.0) );
        REQUIRE( twist2.get<blades::e12>() == Approx(3.0) );
        REQUIRE( twist2.get<blades::e1i>() == Approx(4.0) );
        REQUIRE( twist2.get<blades::e2i>() == Approx(5.0) );
        REQUIRE( twist2.get<blades::e3i>() == Approx(6.0) );
    }

    SECTION( "with translation" )
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Motor<double> motor(translator);

        Twist<double> twist2 = twist.transform(motor);

        REQUIRE( twist2.get<blades::e23>() == Approx(1.0) );
        REQUIRE( twist2.get<blades::e13>() == Approx(2.0) );
        REQUIRE( twist2.get<blades::e12>() == Approx(3.0) );
        REQUIRE( twist2.get<blades::e1i>() == Approx(6.0) );
        REQUIRE( twist2.get<blades::e2i>() == Approx(6.0) );
        REQUIRE( twist2.get<blades::e3i>() == Approx(6.0) );
    }

    SECTION( "with rotation" )
    {
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(rotor);

        Twist<double> twist2 = twist.transform(motor);

        REQUIRE( twist2.get<blades::e23>() == Approx(2.0) );
        REQUIRE( twist2.get<blades::e13>() == Approx(-1.0) );
        REQUIRE( twist2.get<blades::e12>() == Approx(3.0) );
        REQUIRE( twist2.get<blades::e1i>() == Approx(-5.0) );
        REQUIRE( twist2.get<blades::e2i>() == Approx(4.0) );
        REQUIRE( twist2.get<blades::e3i>() == Approx(6.0) );
    }

    SECTION( "with translation & rotation" )
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(translator, rotor);

        Twist<double> twist2 = twist.transform(motor);

        REQUIRE( twist2.get<blades::e23>() == Approx(2.0) );
        REQUIRE( twist2.get<blades::e13>() == Approx(-1.0) );
        REQUIRE( twist2.get<blades::e12>() == Approx(3.0) );
        REQUIRE( twist2.get<blades::e1i>() == Approx(-6.0) );
        REQUIRE( twist2.get<blades::e2i>() == Approx(6.0) );
        REQUIRE( twist2.get<blades::e3i>() == Approx(6.0) );
    }
}


TEST_CASE( "Twists addition", "[Twist]" )
{

    Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Twist<double> twist2({ 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 });

    twist += twist2;

    REQUIRE( twist.get<blades::e23>() == Approx(11.0) );
    REQUIRE( twist.get<blades::e13>() == Approx(22.0) );
    REQUIRE( twist.get<blades::e12>() == Approx(33.0) );
    REQUIRE( twist.get<blades::e1i>() == Approx(44.0) );
    REQUIRE( twist.get<blades::e2i>() == Approx(55.0) );
    REQUIRE( twist.get<blades::e3i>() == Approx(66.0) );
}


TEST_CASE( "Wrench commutation", "[Twist]" )
{
    Twist<double> twist({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    Wrench<double> wrench2 = twist.commute(wrench);

    REQUIRE( wrench2.get<blades::e23>() == Approx(0.0) );
    REQUIRE( wrench2.get<blades::e13>() == Approx(0.0) );
    REQUIRE( wrench2.get<blades::e12>() == Approx(0.0) );
    REQUIRE( wrench2.get<blades::e01>() == Approx(27.0) );
    REQUIRE( wrench2.get<blades::e02>() == Approx(-6.0) );
    REQUIRE( wrench2.get<blades::e03>() == Approx(-13.0) );
}
