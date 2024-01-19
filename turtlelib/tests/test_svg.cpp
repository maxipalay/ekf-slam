#include <catch2/catch_test_macros.hpp>
#include "turtlelib/svg.hpp"
#include <sstream>

namespace turtlelib {

    TEST_CASE("Svg generation", "[svg]") {

        auto expected_output =  "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
                                "<defs>\n"
                                "<marker\n"
                                "        style=\"overflow:visible\"\n"
                                "        id=\"Arrow1Send\"\n"
                                "        refX=\"0.0\"\n"
                                "        refY=\"0.0\"\n"
                                "        orient=\"auto\">\n"
                                "        <path\n"
                                "            transform=\"scale(-0.2) translate(6,0)\"\n"
                                "            style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n"
                                "            d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n"
                                "            />\n"
                                "        </marker>\n"
                                "</defs>\n"
                                "<g>\n"
                                "<line x1=\"408\" x2=\"504\" y1=\"528\" y2=\"528\" stroke=\"red\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n"
                                "<line x1=\"408\" x2=\"408\" y1=\"528\" y2=\"432\" stroke=\"green\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n"
                                "<text x=\"408\" y=\"528\">{a}</text>\n"
                                "</g>\n"
                                "<circle cx=\"504\" cy=\"432\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\" />\n"
                                "<line x1=\"600\" x2=\"696\" y1=\"336\" y2=\"240\" stroke=\"purple\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n"
                                "</svg>";

        Svg mysvg = Svg();

        mysvg.addCoordinateAxes(Transform2D({0,0},0.0), std::string{"a"});
        mysvg.addPoint({1,1}, std::string{"purple"});
        mysvg.addVector({2,2},{3,3}, std::string{"purple"});

        std::stringstream ss{""};
        ss << mysvg.getOutput().str();
        
        REQUIRE(ss.str() == expected_output);
    }

    

}