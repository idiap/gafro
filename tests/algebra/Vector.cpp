#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;


TEST_CASE( "Vector creation", "[Vector]" )
{
    Vector<double> vector;

    REQUIRE( vector.get<blades::e1>() == Approx(0.0) );
    REQUIRE( vector.get<blades::e2>() == Approx(0.0) );
    REQUIRE( vector.get<blades::e3>() == Approx(0.0) );
}


TEST_CASE( "Vector creation from parameters", "[Vector]" )
{
    Vector<double> vector(1.0, 2.0, 3.0);

    REQUIRE( vector.get<blades::e1>() == Approx(1.0) );
    REQUIRE( vector.get<blades::e2>() == Approx(2.0) );
    REQUIRE( vector.get<blades::e3>() == Approx(3.0) );
}


TEST_CASE( "Vector creation from vector", "[Vector]" )
{
    Vector<double> vector1(1.0, 2.0, 3.0);
    Vector<double> vector2(vector1);

    REQUIRE( vector2.get<blades::e1>() == Approx(1.0) );
    REQUIRE( vector2.get<blades::e2>() == Approx(2.0) );
    REQUIRE( vector2.get<blades::e3>() == Approx(3.0) );
}


TEST_CASE( "Vector creation from Multivector", "[Vector]" )
{
    Multivector<double, blades::e1, blades::e2, blades::e3> mv({ 1.0, 2.0, 3.0 });
    Vector<double> vector(mv);

    REQUIRE( vector.get<blades::e1>() == Approx(1.0) );
    REQUIRE( vector.get<blades::e2>() == Approx(2.0) );
    REQUIRE( vector.get<blades::e3>() == Approx(3.0) );
}


TEST_CASE( "Vector creation from expression", "[Vector]" )
{

    Vector<double> vector1( 1.0, 2.0, 3.0 );
    Vector<double> vector2( 10.0, 20.0, 30.0 );

    Sum<Vector<double>, Vector<double>, detail::AdditionOperator> sum(vector1, vector2);

    Vector<double> vector(sum);

    REQUIRE( vector.get<blades::e1>() == Approx(11.0) );
    REQUIRE( vector.get<blades::e2>() == Approx(22.0) );
    REQUIRE( vector.get<blades::e3>() == Approx(33.0) );
}
