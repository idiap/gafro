#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Plane creation from points", "[Plane]" )
{
    SECTION( "from origin, x normal" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(0.0, 1.0, 0.0);
        Point<double> p3(0.0, 0.0, 1.0);

        Plane<double> plane(p1, p2, p3);

        REQUIRE( plane.get<blades::e123i>() == Approx(0.0) );
        REQUIRE( plane.get<blades::e012i>() == Approx(0.0) );
        REQUIRE( plane.get<blades::e023i>() == Approx(1.0) );
        REQUIRE( plane.get<blades::e013i>() == Approx(0.0) );

        auto normal = plane.getNormal();

        REQUIRE( normal.get<blades::e1>() == Approx(1.0) );
        REQUIRE( normal.get<blades::e2>() == Approx(0.0) );
        REQUIRE( normal.get<blades::e3>() == Approx(0.0) );
    }


    SECTION( "arbitrary" )
    {
        Point<double> p1(1.0, 0.0, 0.0);
        Point<double> p2(0.0, 1.0, 0.0);
        Point<double> p3(0.0, 2.0, 3.0);

        Plane<double> plane(p1, p2, p3);

        REQUIRE( plane.get<blades::e123i>() == Approx(3.0) );
        REQUIRE( plane.get<blades::e012i>() == Approx(-1.0) );
        REQUIRE( plane.get<blades::e023i>() == Approx(3.0) );
        REQUIRE( plane.get<blades::e013i>() == Approx(-3.0) );

        auto normal = plane.getNormal();

        REQUIRE( normal.get<blades::e1>() == Approx(3.0) );
        REQUIRE( normal.get<blades::e2>() == Approx(3.0) );
        REQUIRE( normal.get<blades::e3>() == Approx(-1.0) );
    }
}


TEST_CASE( "Plane creation from plane", "[Plane]" )
{
    Point<double> p1(0.0, 0.0, 0.0);
    Point<double> p2(0.0, 1.0, 0.0);
    Point<double> p3(0.0, 0.0, 1.0);

    Plane<double> plane(p1, p2, p3);
    Plane<double> plane2(plane);

    REQUIRE( plane2.get<blades::e123i>() == Approx(0.0) );
    REQUIRE( plane2.get<blades::e012i>() == Approx(0.0) );
    REQUIRE( plane2.get<blades::e023i>() == Approx(1.0) );
    REQUIRE( plane2.get<blades::e013i>() == Approx(0.0) );
}


TEST_CASE( "Plane creation from Multivector", "[Plane]" )
{
    Multivector<double, blades::e123i, blades::e012i, blades::e023i, blades::e013i> mv({ 1.0, 2.0, 3.0, 4.0 });
    Plane<double> plane(mv);

    REQUIRE( plane.get<blades::e123i>() == Approx(1.0) );
    REQUIRE( plane.get<blades::e012i>() == Approx(2.0) );
    REQUIRE( plane.get<blades::e023i>() == Approx(3.0) );
    REQUIRE( plane.get<blades::e013i>() == Approx(4.0) );
}
