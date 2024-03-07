#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Circle creation from points", "[Circle]" )
{
    SECTION( "unit, at origin" )
    {
        Point<double> p1(1.0, 0.0, 0.0);
        Point<double> p2(0.0, 1.0, 0.0);
        Point<double> p3(-1.0, 0.0, 0.0);

        Circle<double> circle(p1, p2, p3);

        REQUIRE( circle.get<blades::e123>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e12i>() == Approx(1.0) );
        REQUIRE( circle.get<blades::e13i>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e23i>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e012>() == Approx(2.0) );
        REQUIRE( circle.get<blades::e013>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e023>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e01i>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e02i>() == Approx(0.0) );
        REQUIRE( circle.get<blades::e03i>() == Approx(0.0) );

        // TODO: This returns NaN
        // REQUIRE( circle.getRadius() == Approx(1.0) );

        auto center = circle.getCenter();

        REQUIRE( center.get<blades::e1>() == Approx(0.0) );
        REQUIRE( center.get<blades::e2>() == Approx(0.0) );
        REQUIRE( center.get<blades::e3>() == Approx(0.0) );
        REQUIRE( center.get<blades::ei>() == Approx(0.0) );
        REQUIRE( center.get<blades::e0>() == Approx(1.0) );

        auto plane = circle.getPlane();

        REQUIRE( plane.get<blades::e123i>() == Approx(0.0) );
        REQUIRE( plane.get<blades::e012i>() == Approx(2.0) );
        REQUIRE( plane.get<blades::e023i>() == Approx(0.0) );
        REQUIRE( plane.get<blades::e013i>() == Approx(0.0) );
    }
}


TEST_CASE( "Circle creation from circle", "[Circle]" )
{
    Point<double> p1(1.0, 0.0, 0.0);
    Point<double> p2(0.0, 1.0, 0.0);
    Point<double> p3(-1.0, 0.0, 0.0);

    Circle<double> circle(p1, p2, p3);
    Circle<double> circle2(circle);

    REQUIRE( circle2.get<blades::e123>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e12i>() == Approx(1.0) );
    REQUIRE( circle2.get<blades::e13i>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e23i>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e012>() == Approx(2.0) );
    REQUIRE( circle2.get<blades::e013>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e023>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e01i>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e02i>() == Approx(0.0) );
    REQUIRE( circle2.get<blades::e03i>() == Approx(0.0) );
}


TEST_CASE( "Circle creation from Multivector", "[Circle]" )
{
    Multivector<double, blades::e123, blades::e12i, blades::e13i, blades::e23i, blades::e012, blades::e013, blades::e023,
                blades::e01i, blades::e02i, blades::e03i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 });
    Circle<double> circle(mv);

    REQUIRE( circle.get<blades::e123>() == Approx(1.0) );
    REQUIRE( circle.get<blades::e12i>() == Approx(2.0) );
    REQUIRE( circle.get<blades::e13i>() == Approx(3.0) );
    REQUIRE( circle.get<blades::e23i>() == Approx(4.0) );
    REQUIRE( circle.get<blades::e012>() == Approx(5.0) );
    REQUIRE( circle.get<blades::e013>() == Approx(6.0) );
    REQUIRE( circle.get<blades::e023>() == Approx(7.0) );
    REQUIRE( circle.get<blades::e01i>() == Approx(8.0) );
    REQUIRE( circle.get<blades::e02i>() == Approx(9.0) );
    REQUIRE( circle.get<blades::e03i>() == Approx(10.0) );
}
