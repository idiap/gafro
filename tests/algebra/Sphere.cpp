#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Sphere creation from center and radius", "[Sphere]" )
{
    SECTION( "unit, at origin" )
    {
        Point<double> center(0.0, 0.0, 0.0);

        Sphere<double> sphere(center, 1.0);

        REQUIRE( sphere.get<blades::e123i>() == Approx(0.5) );
        REQUIRE( sphere.get<blades::e0123>() == Approx(-1.0) );
        REQUIRE( sphere.get<blades::e012i>() == Approx(0.0) );
        REQUIRE( sphere.get<blades::e023i>() == Approx(0.0) );
        REQUIRE( sphere.get<blades::e013i>() == Approx(0.0) );

        REQUIRE( sphere.getRadius() == Approx(1.0) );

        auto centerResult = sphere.getCenter();

        REQUIRE( centerResult.get<blades::e1>() == Approx(0.0) );
        REQUIRE( centerResult.get<blades::e2>() == Approx(0.0) );
        REQUIRE( centerResult.get<blades::e3>() == Approx(0.0) );
        REQUIRE( centerResult.get<blades::ei>() == Approx(0.0) );
        REQUIRE( centerResult.get<blades::e0>() == Approx(1.0) );
    }

    SECTION( "other" )
    {
        Point<double> center(1.0, 2.0, 3.0);

        Sphere<double> sphere(center, 2.0);

        REQUIRE( sphere.get<blades::e123i>() == Approx(-5.0) );
        REQUIRE( sphere.get<blades::e0123>() == Approx(-1.0) );
        REQUIRE( sphere.get<blades::e012i>() == Approx(-3.0) );
        REQUIRE( sphere.get<blades::e023i>() == Approx(-1.0) );
        REQUIRE( sphere.get<blades::e013i>() == Approx(2.0) );

        REQUIRE( sphere.getRadius() == Approx(2.0) );

        auto centerResult = sphere.getCenter();

        REQUIRE( centerResult.get<blades::e1>() == Approx(1.0) );
        REQUIRE( centerResult.get<blades::e2>() == Approx(2.0) );
        REQUIRE( centerResult.get<blades::e3>() == Approx(3.0) );
        REQUIRE( centerResult.get<blades::ei>() == Approx(7.0) );
        REQUIRE( centerResult.get<blades::e0>() == Approx(1.0) );
    }
}


TEST_CASE( "Sphere creation from points", "[Sphere]" )
{
    Point<double> p1(1.0, 0.0, 0.0);
    Point<double> p2(0.0, 1.0, 0.0);
    Point<double> p3(-1.0, 0.0, 0.0);
    Point<double> p4(0.0, 0.0, 1.0);

    Sphere<double> sphere(p1, p2, p3, p4);

    REQUIRE( sphere.get<blades::e123i>() == Approx(-1.0) );
    REQUIRE( sphere.get<blades::e0123>() == Approx(2.0) );
    REQUIRE( sphere.get<blades::e012i>() == Approx(0.0) );
    REQUIRE( sphere.get<blades::e023i>() == Approx(0.0) );
    REQUIRE( sphere.get<blades::e013i>() == Approx(0.0) );

    REQUIRE( sphere.getRadius() == Approx(1.0) );

    auto centerResult = sphere.getCenter();

    REQUIRE( centerResult.get<blades::e1>() == Approx(0.0) );
    REQUIRE( centerResult.get<blades::e2>() == Approx(0.0) );
    REQUIRE( centerResult.get<blades::e3>() == Approx(0.0) );
    REQUIRE( centerResult.get<blades::ei>() == Approx(0.0) );
    REQUIRE( centerResult.get<blades::e0>() == Approx(1.0) );
}


TEST_CASE( "Sphere creation from sphere", "[Sphere]" )
{
    Point<double> center(0.0, 0.0, 0.0);

    Sphere<double> sphere(center, 1.0);
    Sphere<double> sphere2(sphere);

    REQUIRE( sphere2.get<blades::e123i>() == Approx(0.5) );
    REQUIRE( sphere2.get<blades::e0123>() == Approx(-1.0) );
    REQUIRE( sphere2.get<blades::e012i>() == Approx(0.0) );
    REQUIRE( sphere2.get<blades::e023i>() == Approx(0.0) );
    REQUIRE( sphere2.get<blades::e013i>() == Approx(0.0) );
}


TEST_CASE( "Sphere creation from Multivector", "[Sphere]" )
{
    Multivector<double, blades::e123i, blades::e0123, blades::e012i, blades::e023i, blades::e013i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    Sphere<double> sphere(mv);

    REQUIRE( sphere.get<blades::e123i>() == Approx(1.0) );
    REQUIRE( sphere.get<blades::e0123>() == Approx(2.0) );
    REQUIRE( sphere.get<blades::e012i>() == Approx(3.0) );
    REQUIRE( sphere.get<blades::e023i>() == Approx(4.0) );
    REQUIRE( sphere.get<blades::e013i>() == Approx(5.0) );
}
