#include <catch.hpp>
#include <gafro/gafro.hpp>

using namespace gafro;

TEST_CASE("Default link", "[Link]")
{
    Link<double> link;

    REQUIRE(link.getMass() == Approx(0.0));

    const Translator<double> &centerOfMass = link.getCenterOfMass();

    REQUIRE(centerOfMass.get<blades::scalar>() == Approx(1.0));
    REQUIRE(centerOfMass.get<blades::e1i>() == Approx(0.0));
    REQUIRE(centerOfMass.get<blades::e2i>() == Approx(0.0));
    REQUIRE(centerOfMass.get<blades::e3i>() == Approx(0.0));

    REQUIRE(link.getName() == "");

    REQUIRE(link.getParentJoint() == nullptr);

    const std::vector<const Joint<double> *> &children = link.getChildJoints();

    REQUIRE(children.size() == 0);

    const typename Motor<double>::Generator &axis = link.getAxis();

    Rotor<double>::Generator rotorGenerator = axis.getRotorGenerator();
    Translator<double>::Generator translatorGenerator = axis.getTranslatorGenerator();

    REQUIRE(rotorGenerator.e23() == Approx(0.0));
    REQUIRE(rotorGenerator.e13() == Approx(0.0));
    REQUIRE(rotorGenerator.e12() == Approx(0.0));

    REQUIRE(translatorGenerator.x() == Approx(0.0));
    REQUIRE(translatorGenerator.y() == Approx(0.0));
    REQUIRE(translatorGenerator.z() == Approx(0.0));

    SECTION("mass")
    {
        link.setMass(10.0);

        REQUIRE(link.getMass() == Approx(10.0));
    }

    SECTION("center of mass")
    {
        Translator<double>::Generator generator({ 1.0, 2.0, 3.0 });
        Translator<double> translator(generator);

        link.setCenterOfMass(translator);

        const Translator<double> &centerOfMass = link.getCenterOfMass();

        REQUIRE(centerOfMass.get<blades::scalar>() == Approx(translator.get<blades::scalar>()));
        REQUIRE(centerOfMass.get<blades::e1i>() == Approx(translator.get<blades::e1i>()));
        REQUIRE(centerOfMass.get<blades::e2i>() == Approx(translator.get<blades::e2i>()));
        REQUIRE(centerOfMass.get<blades::e3i>() == Approx(translator.get<blades::e3i>()));
    }

    SECTION("inertia")
    {
        Eigen::Matrix<double, 3, 3> tensor = Eigen::Matrix<double, 3, 3>::Identity();
        Inertia<double> inertia(10.0, tensor);

        link.setInertia(inertia);

        const Inertia<double> &inertia2 = link.getInertia();

        Eigen::Matrix<double, 6, 6> tensor2 = inertia2.getTensor();

        REQUIRE(tensor2.coeff(0, 0) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(0, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 1) == Approx(1.0).margin(1e-6));
        REQUIRE(tensor2.coeff(1, 2) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 0) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 1) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(2, 2) == Approx(1.0).margin(1e-6));

        REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
        REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
        REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 3; j < 6; ++j)
            {
                REQUIRE(tensor2.coeff(i, j) == Approx(0.0).margin(1e-6));
                REQUIRE(tensor2.coeff(j, i) == Approx(0.0).margin(1e-6));
            }
        }
    }

    SECTION("name")
    {
        link.setName("link1");

        REQUIRE(link.getName() == "link1");
    }

    SECTION("parent joint")
    {
        RevoluteJoint<double> joint;

        link.setParentJoint(&joint);

        REQUIRE(link.getParentJoint() == &joint);
    }

    SECTION("child joints")
    {
        RevoluteJoint<double> joint1;
        RevoluteJoint<double> joint2;

        link.addChildJoint(&joint1);
        link.addChildJoint(&joint2);

        const std::vector<const Joint<double> *> &children = link.getChildJoints();

        REQUIRE(children.size() == 2);
        REQUIRE(children[0] == &joint1);
        REQUIRE(children[1] == &joint2);
    }

    SECTION("axis")
    {
        Motor<double>::Generator generator({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        link.setAxis(generator);

        const typename Motor<double>::Generator &axis = link.getAxis();

        Rotor<double>::Generator rotorGenerator1 = axis.getRotorGenerator();
        Translator<double>::Generator translatorGenerator1 = axis.getTranslatorGenerator();

        Rotor<double>::Generator rotorGenerator2 = generator.getRotorGenerator();
        Translator<double>::Generator translatorGenerator2 = generator.getTranslatorGenerator();

        REQUIRE(rotorGenerator2.e23() == Approx(rotorGenerator1.e23()));
        REQUIRE(rotorGenerator2.e13() == Approx(rotorGenerator1.e13()));
        REQUIRE(rotorGenerator2.e12() == Approx(rotorGenerator1.e12()));

        REQUIRE(translatorGenerator2.x() == Approx(translatorGenerator1.x()));
        REQUIRE(translatorGenerator2.y() == Approx(translatorGenerator1.y()));
        REQUIRE(translatorGenerator2.z() == Approx(translatorGenerator1.z()));
    }
}

TEST_CASE("Link creation from link", "[Link]")
{
    Link<double> link;

    Translator<double>::Generator centerOfMassGenerator({ 1.0, 2.0, 3.0 });
    Translator<double> translator(centerOfMassGenerator);

    link.setMass(10.0);
    link.setCenterOfMass(translator);

    Eigen::Matrix<double, 3, 3> tensor = Eigen::Matrix<double, 3, 3>::Identity();
    Inertia<double> inertia(10.0, tensor);

    link.setInertia(inertia);

    link.setName("link1");

    RevoluteJoint<double> joint;
    link.setParentJoint(&joint);

    RevoluteJoint<double> joint1;
    RevoluteJoint<double> joint2;

    link.addChildJoint(&joint1);
    link.addChildJoint(&joint2);

    Motor<double>::Generator axisGenerator({ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
    link.setAxis(axisGenerator);

    Link<double> link2(std::move(link));

    const Translator<double> &centerOfMass = link2.getCenterOfMass();

    REQUIRE(centerOfMass.get<blades::scalar>() == Approx(translator.get<blades::scalar>()));
    REQUIRE(centerOfMass.get<blades::e1i>() == Approx(translator.get<blades::e1i>()));
    REQUIRE(centerOfMass.get<blades::e2i>() == Approx(translator.get<blades::e2i>()));
    REQUIRE(centerOfMass.get<blades::e3i>() == Approx(translator.get<blades::e3i>()));

    const Inertia<double> &inertia2 = link2.getInertia();

    REQUIRE(link.getMass() == Approx(10.0));

    Eigen::Matrix<double, 6, 6> tensor2 = inertia2.getTensor();

    REQUIRE(tensor2.coeff(0, 0) == Approx(1.0).margin(1e-6));
    REQUIRE(tensor2.coeff(0, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(0, 2) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(1, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(1, 1) == Approx(1.0).margin(1e-6));
    REQUIRE(tensor2.coeff(1, 2) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(2, 0) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(2, 1) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(2, 2) == Approx(1.0).margin(1e-6));

    REQUIRE(tensor2.coeff(3, 3) == Approx(10.0).margin(1e-6));
    REQUIRE(tensor2.coeff(3, 4) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(3, 5) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(4, 3) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(4, 4) == Approx(10.0).margin(1e-6));
    REQUIRE(tensor2.coeff(4, 5) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(5, 3) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(5, 4) == Approx(0.0).margin(1e-6));
    REQUIRE(tensor2.coeff(5, 5) == Approx(10.0).margin(1e-6));

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 3; j < 6; ++j)
        {
            REQUIRE(tensor2.coeff(i, j) == Approx(0.0).margin(1e-6));
            REQUIRE(tensor2.coeff(j, i) == Approx(0.0).margin(1e-6));
        }
    }

    REQUIRE(link.getMass() == Approx(10.0));
    REQUIRE(link2.getName() == "link1");
    REQUIRE(link2.getParentJoint() == &joint);

    const std::vector<const Joint<double> *> &children = link2.getChildJoints();

    REQUIRE(children.size() == 2);
    REQUIRE(children[0] == &joint1);
    REQUIRE(children[1] == &joint2);

    const typename Motor<double>::Generator &axis = link2.getAxis();

    Rotor<double>::Generator rotorGenerator1 = axis.getRotorGenerator();
    Translator<double>::Generator translatorGenerator1 = axis.getTranslatorGenerator();

    Rotor<double>::Generator rotorGenerator2 = axisGenerator.getRotorGenerator();
    Translator<double>::Generator translatorGenerator2 = axisGenerator.getTranslatorGenerator();

    REQUIRE(rotorGenerator2.e23() == Approx(rotorGenerator1.e23()));
    REQUIRE(rotorGenerator2.e13() == Approx(rotorGenerator1.e13()));
    REQUIRE(rotorGenerator2.e12() == Approx(rotorGenerator1.e12()));

    REQUIRE(translatorGenerator2.x() == Approx(translatorGenerator1.x()));
    REQUIRE(translatorGenerator2.y() == Approx(translatorGenerator1.y()));
    REQUIRE(translatorGenerator2.z() == Approx(translatorGenerator1.z()));
}
