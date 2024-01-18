#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

/// \file
/// \brief Generation of simple svg for visualization of vectors and points.
#include <sstream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    constexpr auto svg_header = "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
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
                                "</defs>\n";

    class Svg
    {
        private:
            std::stringstream ss;
            Transform2D tf{{500.0,500.0}};
            int size_axis{100};

        public:
            Svg(){
                ss << svg_header;
            }

            void addVector(Point2D p, Vector2D v);
            void addPoint(Point2D p);
            void addCoordinateAxes(Point2D p, char name);
            
            std::stringstream& getOutput(){
                ss << "</svg>";
                return (ss);
            }
    };

}

#endif