#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

/// \file
/// \brief Generation of simple svg for visualization of vectors and points.
#include <sstream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief svg header (static text)
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

    /// \brief class that represents an svg document
    class Svg
    {
        private:
            /// \brief stream of characters that compose the svg file
            std::stringstream ss;
            /// \brief transform for centering content on the page 
            Transform2D tf{{408.0,528.0},0.0};
            /// \brief axis size for each drawn axis 
            double size_axis{100};
            /// \brief scaling for the output (1 inch = 96 pixels)
            double scaling{96};

        public:
            /// \brief Create an svg class 
            Svg(){
                ss << svg_header;
            }

            /// \brief add vector to svg file
            /// \param p - origin of the vector
            /// \param v - vector
            /// \param color - the color in which to draw
            void addVector(Point2D p, Vector2D v, const std::string& color);
            
            /// \brief add a point to svg file
            /// \param p - point to draw
            /// \param color - the color in which to draw
            void addPoint(Point2D p, const std::string& color);
            
            /// \brief add coordinate axes to svg file
            /// \param frame - 2D transform
            /// \param name - name of the coordinate frame
            void addCoordinateAxes(Transform2D frame, const std::string& name);
            
            /// \brief get the full svg file output as stringstream
            /// \return output file as stringstream 
            std::stringstream& getOutput(){
                ss << "</svg>";
                return (ss);
            }
    };

}

#endif