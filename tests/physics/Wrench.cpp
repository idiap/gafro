#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("Wrench creation", "[Wrench]")
{
    Wrench<double> wrench;

    REQUIRE(wrench.get<blades::e23>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(0.0));

    auto mv = wrench.multivector();

    REQUIRE(mv.bits().size() == wrench.bits().size());

    REQUIRE(mv.get<blades::e23>() == Approx(0.0));
    REQUIRE(mv.get<blades::e13>() == Approx(0.0));
    REQUIRE(mv.get<blades::e12>() == Approx(0.0));
    REQUIRE(mv.get<blades::e01>() == Approx(0.0));
    REQUIRE(mv.get<blades::e02>() == Approx(0.0));
    REQUIRE(mv.get<blades::e03>() == Approx(0.0));
}

TEST_CASE("Zero wrench", "[Wrench]")
{
    Wrench<double> wrench = Wrench<double>::Zero();

    REQUIRE(wrench.get<blades::e23>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(0.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(0.0));
}

TEST_CASE("Wrench creation from parameters", "[Wrench]")
{
    Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    REQUIRE(wrench.get<blades::e23>() == Approx(1.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(2.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(3.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(4.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(5.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(6.0));

    auto mv = wrench.multivector();

    REQUIRE(mv.bits().size() == wrench.bits().size());

    REQUIRE(mv.get<blades::e23>() == Approx(1.0));
    REQUIRE(mv.get<blades::e13>() == Approx(2.0));
    REQUIRE(mv.get<blades::e12>() == Approx(3.0));
    REQUIRE(mv.get<blades::e01>() == Approx(4.0));
    REQUIRE(mv.get<blades::e02>() == Approx(5.0));
    REQUIRE(mv.get<blades::e03>() == Approx(6.0));
}

TEST_CASE("Wrench creation from wrench", "[Wrench]")
{
    Wrench<double> wrench1({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Wrench<double> wrench2(wrench1);

    REQUIRE(wrench2.get<blades::e23>() == Approx(1.0));
    REQUIRE(wrench2.get<blades::e13>() == Approx(2.0));
    REQUIRE(wrench2.get<blades::e12>() == Approx(3.0));
    REQUIRE(wrench2.get<blades::e01>() == Approx(4.0));
    REQUIRE(wrench2.get<blades::e02>() == Approx(5.0));
    REQUIRE(wrench2.get<blades::e03>() == Approx(6.0));
}

TEST_CASE("Wrench creation from Multivector", "[Wrench]")
{
    Multivector<double, blades::e23, blades::e13, blades::e12, blades::e01, blades::e02, blades::e03> mv({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Wrench<double> wrench(mv);

    REQUIRE(wrench.get<blades::e23>() == Approx(1.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(2.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(3.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(4.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(5.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(6.0));
}

TEST_CASE("Wrench transform by motor", "[Wrench]")
{
    Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    SECTION("default")
    {
        Motor<double> motor;

        Wrench<double> wrench2 = wrench.transform(motor);

        REQUIRE(wrench2.get<blades::e23>() == Approx(1.0));
        REQUIRE(wrench2.get<blades::e13>() == Approx(2.0));
        REQUIRE(wrench2.get<blades::e12>() == Approx(3.0));
        REQUIRE(wrench2.get<blades::e01>() == Approx(4.0));
        REQUIRE(wrench2.get<blades::e02>() == Approx(5.0));
        REQUIRE(wrench2.get<blades::e03>() == Approx(6.0));
    }

    SECTION("with translation")
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Motor<double> motor(translator);

        Wrench<double> wrench2 = wrench.transform(motor);

        REQUIRE(wrench2.get<blades::e23>() == Approx(-4.0));
        REQUIRE(wrench2.get<blades::e13>() == Approx(-2.0));
        REQUIRE(wrench2.get<blades::e12>() == Approx(3.0));
        REQUIRE(wrench2.get<blades::e01>() == Approx(4.0));
        REQUIRE(wrench2.get<blades::e02>() == Approx(5.0));
        REQUIRE(wrench2.get<blades::e03>() == Approx(6.0));
    }

    SECTION("with rotation")
    {
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(rotor);

        Wrench<double> wrench2 = wrench.transform(motor);

        REQUIRE(wrench2.get<blades::e23>() == Approx(2.0));
        REQUIRE(wrench2.get<blades::e13>() == Approx(-1.0));
        REQUIRE(wrench2.get<blades::e12>() == Approx(3.0));
        REQUIRE(wrench2.get<blades::e01>() == Approx(-5.0));
        REQUIRE(wrench2.get<blades::e02>() == Approx(4.0));
        REQUIRE(wrench2.get<blades::e03>() == Approx(6.0));
    }

    SECTION("with translation & rotation")
    {
        Translator<double> translator(Translator<double>::Generator({ 0.0, 0.0, 1.0 }));
        Rotor<double> rotor(Eigen::Matrix<double, 3, 1>({ 0.0, 0.0, 1.0 }), M_PI / 2.0);
        Motor<double> motor(translator, rotor);

        Wrench<double> wrench2 = wrench.transform(motor);

        REQUIRE(wrench2.get<blades::e23>() == Approx(-2.0));
        REQUIRE(wrench2.get<blades::e13>() == Approx(4.0));
        REQUIRE(wrench2.get<blades::e12>() == Approx(3.0));
        REQUIRE(wrench2.get<blades::e01>() == Approx(-5.0));
        REQUIRE(wrench2.get<blades::e02>() == Approx(4.0));
        REQUIRE(wrench2.get<blades::e03>() == Approx(6.0));
    }
}

TEST_CASE("Wrenchs addition", "[Wrench]")
{
    Wrench<double> wrench({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    Wrench<double> wrench2({ 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 });

    wrench += wrench2;

    REQUIRE(wrench.get<blades::e23>() == Approx(11.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(22.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(33.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(44.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(55.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(66.0));
}

TEST_CASE("Wrenchs substraction", "[Wrench]")
{
    Wrench<double> wrench({ 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 });
    Wrench<double> wrench2({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

    wrench -= wrench2;

    REQUIRE(wrench.get<blades::e23>() == Approx(9.0));
    REQUIRE(wrench.get<blades::e13>() == Approx(18.0));
    REQUIRE(wrench.get<blades::e12>() == Approx(27.0));
    REQUIRE(wrench.get<blades::e01>() == Approx(36.0));
    REQUIRE(wrench.get<blades::e02>() == Approx(45.0));
    REQUIRE(wrench.get<blades::e03>() == Approx(54.0));
}
