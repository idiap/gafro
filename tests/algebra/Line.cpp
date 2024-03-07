#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Line creation from points", "[Line]" )
{
    SECTION( "from origin" )
    {
        Point<double> p1(0.0, 0.0, 0.0);
        Point<double> p2(1.0, 2.0, 3.0);

        Line<double> line(p1, p2);

        REQUIRE( line.get<blades::e12i>() == Approx(0.0) );
        REQUIRE( line.get<blades::e13i>() == Approx(0.0) );
        REQUIRE( line.get<blades::e23i>() == Approx(0.0) );
        REQUIRE( line.get<blades::e01i>() == Approx(1.0) );
        REQUIRE( line.get<blades::e02i>() == Approx(2.0) );
        REQUIRE( line.get<blades::e03i>() == Approx(3.0) );
    }

    SECTION( "arbitrary" )
    {
        Point<double> p1(1.0, 2.0, 3.0);
        Point<double> p2(4.0, 5.0, 6.0);

        Line<double> line(p1, p2);

        REQUIRE( line.get<blades::e12i>() == Approx(-3.0) );
        REQUIRE( line.get<blades::e13i>() == Approx(-6.0) );
        REQUIRE( line.get<blades::e23i>() == Approx(-3.0) );
        REQUIRE( line.get<blades::e01i>() == Approx(3.0) );
        REQUIRE( line.get<blades::e02i>() == Approx(3.0) );
        REQUIRE( line.get<blades::e03i>() == Approx(3.0) );
    }
}


TEST_CASE( "Line creation from line", "[Line]" )
{
    Point<double> p1(1.0, 2.0, 3.0);
    Point<double> p2(4.0, 5.0, 6.0);

    Line<double> line(p1, p2);
    Line<double> line2(line);

    REQUIRE( line2.get<blades::e12i>() == Approx(-3.0) );
    REQUIRE( line2.get<blades::e13i>() == Approx(-6.0) );
    REQUIRE( line2.get<blades::e23i>() == Approx(-3.0) );
    REQUIRE( line2.get<blades::e01i>() == Approx(3.0) );
    REQUIRE( line2.get<blades::e02i>() == Approx(3.0) );
    REQUIRE( line2.get<blades::e03i>() == Approx(3.0) );
}


TEST_CASE( "Line creation from Multivector", "[Line]" )
{
    Multivector<double, blades::e12i, blades::e13i, blades::e23i, blades::e01i, blades::e02i, blades::e03i> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Line<double> line(mv);

    REQUIRE( line.get<blades::e12i>() == Approx(1.0) );
    REQUIRE( line.get<blades::e13i>() == Approx(2.0) );
    REQUIRE( line.get<blades::e23i>() == Approx(3.0) );
    REQUIRE( line.get<blades::e01i>() == Approx(4.0) );
    REQUIRE( line.get<blades::e02i>() == Approx(5.0) );
    REQUIRE( line.get<blades::e03i>() == Approx(6.0) );
}
