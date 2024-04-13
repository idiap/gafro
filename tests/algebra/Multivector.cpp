#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("Multivector size", "[Multivector]")
{
    REQUIRE(gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::size == 3);
    REQUIRE(gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>::size == 5);
}

TEST_CASE("Multivector bitset", "[Multivector]")
{
    auto bits1 = gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::bits();
    std::array<int, 3> blades1 = bits1.blades();

    REQUIRE(blades1[0] == blades::e1);
    REQUIRE(blades1[1] == blades::e2);
    REQUIRE(blades1[2] == blades::e3);

    auto bits2 = gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>::bits();
    std::array<int, 5> blades2 = bits2.blades();

    REQUIRE(blades2[0] == blades::e0);
    REQUIRE(blades2[1] == blades::e1);
    REQUIRE(blades2[2] == blades::e2);
    REQUIRE(blades2[3] == blades::e3);
    REQUIRE(blades2[4] == blades::ei);
}

TEST_CASE("Multivector blades", "[Multivector]")
{
    std::array<int, 3> blades1 = gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::blades();

    REQUIRE(blades1[0] == blades::e1);
    REQUIRE(blades1[1] == blades::e2);
    REQUIRE(blades1[2] == blades::e3);

    std::array<int, 5> blades2 = gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>::blades();

    REQUIRE(blades2[0] == blades::e0);
    REQUIRE(blades2[1] == blades::e1);
    REQUIRE(blades2[2] == blades::e2);
    REQUIRE(blades2[3] == blades::e3);
    REQUIRE(blades2[4] == blades::ei);
}

TEST_CASE("Multivector has blade", "[Multivector]")
{
    REQUIRE(gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::has(blades::e1));
    REQUIRE(gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::has(blades::e2));
    REQUIRE(gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::has(blades::e3));
    REQUIRE(!gafro::Multivector<double, blades::e1, blades::e2, blades::e3>::has(blades::e0));
}

TEST_CASE("Multivector creation", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv;

    REQUIRE(mv.get<blades::e1>() == Approx(0.0));
    REQUIRE(mv.get<blades::e2>() == Approx(0.0));
    REQUIRE(mv.get<blades::e3>() == Approx(0.0));
    REQUIRE(mv.get<blades::ei>() == Approx(0.0));
    REQUIRE(mv.get<blades::e0>() == Approx(0.0));
}

TEST_CASE("Multivector creation with value", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv(10);

    REQUIRE(mv.get<blades::e1>() == Approx(10.0));
    REQUIRE(mv.get<blades::e2>() == Approx(10.0));
    REQUIRE(mv.get<blades::e3>() == Approx(10.0));
    REQUIRE(mv.get<blades::ei>() == Approx(10.0));
    REQUIRE(mv.get<blades::e0>() == Approx(10.0));
}

TEST_CASE("Multivector creation from parameters", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    REQUIRE(mv.get<blades::e0>() == Approx(1.0));
    REQUIRE(mv.get<blades::e1>() == Approx(2.0));
    REQUIRE(mv.get<blades::e2>() == Approx(3.0));
    REQUIRE(mv.get<blades::e3>() == Approx(4.0));
    REQUIRE(mv.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Multivector creation from multivector", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv2(mv);

    REQUIRE(mv2.get<blades::e0>() == Approx(1.0));
    REQUIRE(mv2.get<blades::e1>() == Approx(2.0));
    REQUIRE(mv2.get<blades::e2>() == Approx(3.0));
    REQUIRE(mv2.get<blades::e3>() == Approx(4.0));
    REQUIRE(mv2.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Multivector creation from expression", "[Multivector]")
{
    typedef gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> MV;

    MV mv1({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    MV mv2({ 10.0, 20.0, 30.0, 40.0, 50.0 });

    Sum<MV, MV, detail::AdditionOperator> sum(mv1, mv2);

    MV mv(sum);

    REQUIRE(mv.get<blades::e0>() == Approx(11.0));
    REQUIRE(mv.get<blades::e1>() == Approx(22.0));
    REQUIRE(mv.get<blades::e2>() == Approx(33.0));
    REQUIRE(mv.get<blades::e3>() == Approx(44.0));
    REQUIRE(mv.get<blades::ei>() == Approx(55.0));
}

TEST_CASE("Multivector creation from expression (alternative)", "[Multivector]")
{
    Point<double> mv1(1.0, 2.0, 3.0);
    Point<double> mv2(10.0, 20.0, 30.0);

    Sum<Point<double>, Point<double>, detail::AdditionOperator> sum(mv1, mv2);

    Point<double> mv(sum);

    REQUIRE(mv.get<blades::e0>() == Approx(2.0));
    REQUIRE(mv.get<blades::e1>() == Approx(11.0));
    REQUIRE(mv.get<blades::e2>() == Approx(22.0));
    REQUIRE(mv.get<blades::e3>() == Approx(33.0));
    REQUIRE(mv.get<blades::ei>() == Approx(707.0));
}

TEST_CASE("Multivector assignation from parameters", "[Multivector]")
{
    typedef gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> MV;

    MV mv = MV::Parameters({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    REQUIRE(mv.get<blades::e0>() == Approx(1.0));
    REQUIRE(mv.get<blades::e1>() == Approx(2.0));
    REQUIRE(mv.get<blades::e2>() == Approx(3.0));
    REQUIRE(mv.get<blades::e3>() == Approx(4.0));
    REQUIRE(mv.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Multivector assignation from multivector", "[Multivector]")
{
    typedef gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> MV;

    MV mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    MV mv2 = mv;

    REQUIRE(mv2.get<blades::e0>() == Approx(1.0));
    REQUIRE(mv2.get<blades::e1>() == Approx(2.0));
    REQUIRE(mv2.get<blades::e2>() == Approx(3.0));
    REQUIRE(mv2.get<blades::e3>() == Approx(4.0));
    REQUIRE(mv2.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Multivector assignation from expression", "[Multivector]")
{
    typedef gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> MV;

    MV mv1({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    MV mv2({ 10.0, 20.0, 30.0, 40.0, 50.0 });

    Sum<MV, MV, detail::AdditionOperator> sum(mv1, mv2);

    MV mv = sum;

    REQUIRE(mv.get<blades::e0>() == Approx(11.0));
    REQUIRE(mv.get<blades::e1>() == Approx(22.0));
    REQUIRE(mv.get<blades::e2>() == Approx(33.0));
    REQUIRE(mv.get<blades::e3>() == Approx(44.0));
    REQUIRE(mv.get<blades::ei>() == Approx(55.0));
}

TEST_CASE("Multivector assignation from expression (alternative)", "[Multivector]")
{
    Point<double> mv1(1.0, 2.0, 3.0);
    Point<double> mv2(10.0, 20.0, 30.0);

    Sum<Point<double>, Point<double>, detail::AdditionOperator> sum(mv1, mv2);

    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv = sum;

    REQUIRE(mv.get<blades::e0>() == Approx(2.0));
    REQUIRE(mv.get<blades::e1>() == Approx(11.0));
    REQUIRE(mv.get<blades::e2>() == Approx(22.0));
    REQUIRE(mv.get<blades::e3>() == Approx(33.0));
    REQUIRE(mv.get<blades::ei>() == Approx(707.0));
}

TEST_CASE("Multivector multiplication by scalar", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    mv *= 2.0;

    REQUIRE(mv.get<blades::e0>() == Approx(2.0));
    REQUIRE(mv.get<blades::e1>() == Approx(4.0));
    REQUIRE(mv.get<blades::e2>() == Approx(6.0));
    REQUIRE(mv.get<blades::e3>() == Approx(8.0));
    REQUIRE(mv.get<blades::ei>() == Approx(10.0));
}

TEST_CASE("Multivector division by scalar", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    mv /= 2.0;

    REQUIRE(mv.get<blades::e0>() == Approx(0.5));
    REQUIRE(mv.get<blades::e1>() == Approx(1.0));
    REQUIRE(mv.get<blades::e2>() == Approx(1.5));
    REQUIRE(mv.get<blades::e3>() == Approx(2.0));
    REQUIRE(mv.get<blades::ei>() == Approx(2.5));
}

TEST_CASE("Multivectors addition", "[Multivector]")
{
    typedef gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> MV;

    MV mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });
    MV mv2({ 10.0, 20.0, 30.0, 40.0, 50.0 });

    mv += mv2;

    REQUIRE(mv.get<blades::e0>() == Approx(11.0));
    REQUIRE(mv.get<blades::e1>() == Approx(22.0));
    REQUIRE(mv.get<blades::e2>() == Approx(33.0));
    REQUIRE(mv.get<blades::e3>() == Approx(44.0));
    REQUIRE(mv.get<blades::ei>() == Approx(55.0));
}

TEST_CASE("Multivector set parameters", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv;
    mv.setParameters({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    REQUIRE(mv.get<blades::e0>() == Approx(1.0));
    REQUIRE(mv.get<blades::e1>() == Approx(2.0));
    REQUIRE(mv.get<blades::e2>() == Approx(3.0));
    REQUIRE(mv.get<blades::e3>() == Approx(4.0));
    REQUIRE(mv.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Multivector get vector", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    auto vector = mv.vector();

    REQUIRE(vector.coeff(0) == Approx(1.0));
    REQUIRE(vector.coeff(1) == Approx(2.0));
    REQUIRE(vector.coeff(2) == Approx(3.0));
    REQUIRE(vector.coeff(3) == Approx(4.0));
    REQUIRE(vector.coeff(4) == Approx(5.0));
}

TEST_CASE("Multivector get reverse", "[Multivector]")
{
    gafro::Multivector<double, blades::e1, blades::e2, blades::e12, blades::e3, blades::e13, blades::e23> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    auto reverse = mv.reverse();

    REQUIRE(reverse.get<blades::e1>() == Approx(1.0));
    REQUIRE(reverse.get<blades::e2>() == Approx(2.0));
    REQUIRE(reverse.get<blades::e12>() == Approx(-3.0));
    REQUIRE(reverse.get<blades::e3>() == Approx(4.0));
    REQUIRE(reverse.get<blades::e13>() == Approx(-5.0));
    REQUIRE(reverse.get<blades::e23>() == Approx(-6.0));
}

TEST_CASE("Multivector get inverse", "[Multivector]")
{
    gafro::Multivector<double, blades::e1, blades::e2, blades::e12, blades::e3, blades::e13, blades::e23> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    auto inverse = mv.inverse();

    REQUIRE(inverse.get<blades::e1>() == Approx(0.010989011));
    REQUIRE(inverse.get<blades::e2>() == Approx(0.021978022));
    REQUIRE(inverse.get<blades::e12>() == Approx(-0.032967033));
    REQUIRE(inverse.get<blades::e3>() == Approx(0.043956044));
    REQUIRE(inverse.get<blades::e13>() == Approx(-0.0549450549));
    REQUIRE(inverse.get<blades::e23>() == Approx(-0.0659340659));
}

TEST_CASE("Multivector get dual", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    auto dual = mv.dual();

    REQUIRE(dual.get<blades::e123i>() == Approx(-5.0));
    REQUIRE(dual.get<blades::e0123>() == Approx(-1.0));
    REQUIRE(dual.get<blades::e012i>() == Approx(-4.0));
    REQUIRE(dual.get<blades::e023i>() == Approx(-2.0));
    REQUIRE(dual.get<blades::e013i>() == Approx(3.0));
}

TEST_CASE("Multivector set blade", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv;
    mv.set<blades::e2>(1.0);

    REQUIRE(mv.get<blades::e1>() == Approx(0.0));
    REQUIRE(mv.get<blades::e2>() == Approx(1.0));
    REQUIRE(mv.get<blades::e3>() == Approx(0.0));
    REQUIRE(mv.get<blades::ei>() == Approx(0.0));
    REQUIRE(mv.get<blades::e0>() == Approx(0.0));
}

TEST_CASE("Multivector get norm", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 5.0, 1.0, 2.0, 3.0, 4.0 });
    REQUIRE(mv.norm() == Approx(5.0990195136));
}

TEST_CASE("Multivector get squared norm", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 5.0, 1.0, 2.0, 3.0, 4.0 });
    REQUIRE(mv.squaredNorm() == Approx(-26.0));
}

TEST_CASE("Multivector get signed norm", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 5.0, 1.0, 2.0, 3.0, 4.0 });
    REQUIRE(mv.signedNorm() == Approx(-5.0990195136));
}

TEST_CASE("Multivector normalize", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 5.0, 1.0, 2.0, 3.0, 4.0 });

    mv.normalize();

    REQUIRE(mv.get<blades::e1>() == Approx(0.1961161351));
    REQUIRE(mv.get<blades::e2>() == Approx(0.3922322703));
    REQUIRE(mv.get<blades::e3>() == Approx(0.5883484054));
    REQUIRE(mv.get<blades::ei>() == Approx(0.7844645406));
    REQUIRE(mv.get<blades::e0>() == Approx(0.9805806757));
}

TEST_CASE("Multivector get normalized copy", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 5.0, 1.0, 2.0, 3.0, 4.0 });

    auto mv2 = mv.normalized();

    REQUIRE(mv2.get<blades::e1>() == Approx(0.1961161351));
    REQUIRE(mv2.get<blades::e2>() == Approx(0.3922322703));
    REQUIRE(mv2.get<blades::e3>() == Approx(0.5883484054));
    REQUIRE(mv2.get<blades::ei>() == Approx(0.7844645406));
    REQUIRE(mv2.get<blades::e0>() == Approx(0.9805806757));

    REQUIRE(mv.get<blades::e0>() == Approx(5.0));
    REQUIRE(mv.get<blades::e1>() == Approx(1.0));
    REQUIRE(mv.get<blades::e2>() == Approx(2.0));
    REQUIRE(mv.get<blades::e3>() == Approx(3.0));
    REQUIRE(mv.get<blades::ei>() == Approx(4.0));
}

TEST_CASE("Multivector cast to Point", "[Multivector]")
{
    gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei> mv({ 1.0, 2.0, 3.0, 4.0, 5.0 });

    auto point = mv.cast<Point<double>>();

    REQUIRE(point.get<blades::e0>() == Approx(1.0));
    REQUIRE(point.get<blades::e1>() == Approx(2.0));
    REQUIRE(point.get<blades::e2>() == Approx(3.0));
    REQUIRE(point.get<blades::e3>() == Approx(4.0));
    REQUIRE(point.get<blades::ei>() == Approx(5.0));
}

TEST_CASE("Random Multivector creation", "[Multivector]")
{
    // We can only check that it compiles and doesn't throw any exception
    auto mv = gafro::Multivector<double, blades::e0, blades::e1, blades::e2, blades::e3, blades::ei>::Random();

    REQUIRE_NOTHROW(mv.get<blades::e1>());
}
