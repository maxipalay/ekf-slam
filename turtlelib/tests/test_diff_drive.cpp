#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib {

    TEST_CASE("FKin - Straight forward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure forward translation
        auto tf = ddrive.FKin(PI/2.0, -PI/2.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(PI*0.05/2.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("FKin - Straight backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure backward translation
        auto tf = ddrive.FKin(-PI/2.0, PI/2.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-PI*0.05/2.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("FKin - Straight forward & backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure forward translation
        ddrive.FKin(PI/4.0, PI/4.0);
        // pure backward translation
        auto tf = ddrive.FKin(0.0, 0.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("FKin - Pure rotation", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // pure rotation
        auto tf = ddrive.FKin((0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI,(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(-PI/2.0, 1.0E-8));
    }

    TEST_CASE("FKin - Pure rotation (inverse)", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // pure rotation
        auto tf = ddrive.FKin(-(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI,-(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1.0E-8));
    }

    TEST_CASE("FKin - Arc forward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc forward translation
        auto tf = ddrive.FKin((0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, -(0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.15, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(-PI/2.0, 1.0E-8));
    }

    TEST_CASE("FKin - Arc backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc backward translation
        auto tf = ddrive.FKin(-(0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, (0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1.0E-8));
    }


    TEST_CASE("FKin - Arc forward & backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc forward translation
        auto tf = ddrive.FKin((0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, -(0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        // arc backward translation
        tf = ddrive.FKin(0.0, 0.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("IKin - Invalid twist", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);
        auto twist = Twist2D{0.1,1.0,1.0};
        
        auto exceptionRaised = false;
        
        try {
            ddrive.IKin(twist);
        } catch (std::logic_error const &) {
            exceptionRaised = true;
        }

        REQUIRE(exceptionRaised);
        
    }

    TEST_CASE("IKin - Straight forward & backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        auto twist = Twist2D{0.0,0.2*2.0*PI,0.0};

        auto speeds = ddrive.IKin(twist);

        REQUIRE_THAT(speeds.left, Catch::Matchers::WithinAbs(2.0*PI, 1.0E-8));
        REQUIRE_THAT(speeds.right, Catch::Matchers::WithinAbs(-2.0*PI, 1.0E-8));

        twist = Twist2D{0.0,-0.2*2.0*PI,0.0};
        speeds = ddrive.IKin(twist);

        REQUIRE_THAT(speeds.left, Catch::Matchers::WithinAbs(-2.0*PI, 1.0E-8));
        REQUIRE_THAT(speeds.right, Catch::Matchers::WithinAbs(2.0*PI, 1.0E-8));
    }

    TEST_CASE("IKin - Rotation CW & CCW", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        auto twist = Twist2D{-PI/2.0,0.0,0.0};

        auto speeds = ddrive.IKin(twist);

        REQUIRE_THAT(speeds.left, Catch::Matchers::WithinAbs(0.1*PI/4.0/(0.2*PI)*2.0*PI, 1.0E-8));
        REQUIRE_THAT(speeds.right, Catch::Matchers::WithinAbs(0.1*PI/4.0/(0.2*PI)*2.0*PI, 1.0E-8));

        twist = Twist2D{PI/2.0,0.0,0.0};

        speeds = ddrive.IKin(twist);

        REQUIRE_THAT(speeds.left, Catch::Matchers::WithinAbs(-0.1*PI/4.0/(0.2*PI)*2.0*PI, 1.0E-8));
        REQUIRE_THAT(speeds.right, Catch::Matchers::WithinAbs(-0.1*PI/4.0/(0.2*PI)*2.0*PI, 1.0E-8));
    }

    // missing arc test case for IKin

}