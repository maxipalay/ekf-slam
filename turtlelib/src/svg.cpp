#include "turtlelib/svg.hpp"

namespace turtlelib {
    
    void Svg::addVector(Point2D p, Vector2D v, const std::string& color){
        ss << "<line x1=\"" << p.x*scaling+tf.translation().x << "\" x2=\"" << v.x*scaling+tf.translation().x;
        ss << "\" y1=\"" << -p.y*scaling+tf.translation().y << "\" y2=\"" << -v.y*scaling+tf.translation().y;
        ss << "\" stroke=\"" << color << "\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
    }
    void Svg::addPoint(Point2D p, const std::string& color){
        ss << "<circle cx=\"" << p.x*scaling+tf.translation().x << "\" cy=\"" << -p.y*scaling+tf.translation().y << "\" r=\"3\" stroke=\"purple\" fill=\"" << color << "\" stroke-width=\"1\" />\n";
    }

    void Svg::addCoordinateAxes(Transform2D frame, const std::string& name){
        Point2D x{1,0};
        Point2D y{0,1};
        auto tf_x = frame(x);
        auto tf_y = frame(y);

        ss << "<g>\n";
        ss << "<line x1=\"" << frame.translation().x*scaling+tf.translation().x << "\" x2=\"" << tf_x.x*scaling+tf.translation().x;
        ss << "\" y1=\"" << -frame.translation().y*scaling+tf.translation().y << "\" y2=\"" << -tf_x.y*scaling+tf.translation().y;
        ss << "\" stroke=\"red\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
        ss << "<line x1=\"" << frame.translation().x*scaling+tf.translation().x << "\" x2=\"" << tf_y.x*scaling+tf.translation().x;
        ss << "\" y1=\"" << -frame.translation().y*scaling+tf.translation().y  << "\" y2=\"" << -tf_y.y*scaling+tf.translation().y;
        ss << "\" stroke=\"green\" stroke-width=\"5\" marker-end=\"url(#Arrow1Send)\" />\n";
        ss << "<text x=\"" << frame.translation().x*scaling+tf.translation().x+text_offset << "\" y=\"" << -frame.translation().y*scaling+tf.translation().y+text_offset << "\">{" << name << "}</text>\n";
        ss << "</g>\n";
    }

}
