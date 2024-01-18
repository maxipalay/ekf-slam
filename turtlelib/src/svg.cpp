#include "turtlelib/svg.hpp"

namespace turtlelib {
    
    void Svg::addVector(Point2D p, Vector2D v){
        auto tf_p = tf(p);
        auto tf_v = tf(v);
        ss << "<line x1=\"" << tf_p.x << "\" x2=\"" << tf_v.x;
        ss << "\" y1=\"" << tf_p.y << "\" y2=\"" << tf_v.y;
        ss << "\" stroke=\"purple\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
    }
    void Svg::addPoint(Point2D p){
        auto tf_p = tf(p);
        ss << "<circle cx=\"" << tf_p.x << "\" cy=\"" << tf_p.y << "\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\" />\n";
    }

    void Svg::addCoordinateAxes(Point2D p, char name){
        auto tf_p = tf(p);
        ss << "<g>\n";
        ss << "<line x1=\"" << tf_p.x << "\" x2=\"" << tf_p.x + size_axis;
        ss << "\" y1=\"" << tf_p.y << "\" y2=\"" << tf_p.y;
        ss << "\" stroke=\"red\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
        ss << "<line x1=\"" << tf_p.x << "\" x2=\"" << tf_p.x;
        ss << "\" y1=\"" << tf_p.y << "\" y2=\"" << tf_p.y + size_axis;
        ss << "\" stroke=\"green\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
        ss << "<text x=\"" << tf_p.x << "\" y=\"" << tf_p.y << "\">{" << name << "}</text>\n";
        ss << "</g>\n";
    }

}
