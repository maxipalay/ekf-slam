#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib {

    TEST_CASE("Straight forward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure forward translation
        auto tf = ddrive.FKin(PI/2.0, PI/2.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(PI*0.05/2.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Straight backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure backward translation
        auto tf = ddrive.FKin(-PI/2.0, -PI/2.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-PI*0.05/2.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Straight forward & backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.05);

        // pure forward translation
        ddrive.FKin(PI/4.0, PI/4.0);
        // pure backward translation
        auto tf = ddrive.FKin(0.0, 0.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    TEST_CASE("Pure rotation", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // pure rotation
        auto tf = ddrive.FKin((0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI,-(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(-PI/2.0, 1.0E-8));
    }

    TEST_CASE("Pure rotation (inverse)", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // pure rotation
        auto tf = ddrive.FKin(-(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI,(0.1*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1.0E-8));
    }

    TEST_CASE("Arc forward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc forward translation
        auto tf = ddrive.FKin((0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, (0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.15, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(-PI/2.0, 1.0E-8));
    }

    TEST_CASE("Arc backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc backward translation
        auto tf = ddrive.FKin(-(0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, -(0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(-0.15, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1.0E-8));
    }


    TEST_CASE("Arc forward & backward", "[diff drive]") {
        auto ddrive = DiffDrive(0.1, 0.2);

        // arc forward translation
        auto tf = ddrive.FKin((0.4*PI/4.0)/(2.0*PI*0.2)*2.0*PI, (0.2*PI/4.0)/(2.0*PI*0.2)*2.0*PI);

        // arc backward translation
        tf = ddrive.FKin(0.0, 0.0);

        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-8));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-8));
    }

    // add tests for IKin

}