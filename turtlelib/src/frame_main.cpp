#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <fstream>

int main()
{
    using namespace std;
    using namespace turtlelib;

    Svg mysvg = Svg();

    // transforms

    cout << "Enter transform T_{a,b}:\n";
    Transform2D Tab = Transform2D{};
    cin >> Tab;
    
    cout << "Enter transform T_{b,c}:\n";
    Transform2D Tbc = Transform2D{};
    cin >> Tbc;

    cout << "T_{a,b}: " << Tab << "\n";

    Transform2D Tba;
    Tba = Tab.inv();
    cout << "T_{b,a}: " << Tba << "\n";

    cout << "T_{b,c}: " << Tbc << "\n";

    Transform2D Tcb = Tbc.inv();
    cout << "T_{c,b}: " << Tcb << "\n";

    Transform2D Tac = Tab * Tbc;
    cout << "T_{a,c}: " << Tac << "\n";

    Transform2D Tca = Tac.inv();
    cout << "T_{c,a}: " << Tca << "\n";

    mysvg.addCoordinateAxes(Transform2D({0,0},0.0), std::string{"a"});
    mysvg.addCoordinateAxes(Tab, std::string{"b"});
    mysvg.addCoordinateAxes(Tac, std::string{"c"});

    // point

    cout << "Enter point p_a:\n";
    Point2D p_a;
    cin >> p_a;

    cout << "p_a: ";
    cout << p_a << "\n";

    Point2D p_b = Tba(p_a);
    cout << "p_b: ";
    cout << p_b << "\n";

    Point2D p_c = Tca(p_a);
    cout << "p_c: ";
    cout << p_c << "\n";

    mysvg.addPoint(p_a, std::string{"purple"});
    mysvg.addPoint(Tab(p_b), std::string{"brown"});
    mysvg.addPoint(Tac(p_c), std::string{"orange"});

    // vector

    cout << "Enter vector v_b:\n";
    Vector2D v_b;
    cin >> v_b;

    auto v_bhat = normalize(v_b);
    cout << "v_bhat: ";
    cout << v_bhat << "\n";

    Vector2D v_a = Tab(v_b);
    cout << "v_a: ";
    cout << v_a << "\n";

    cout << "v_b: ";
    cout << v_b << "\n";

    Vector2D v_c = Tcb(v_b);
    cout << "v_c: ";
    cout << v_c << "\n";

    mysvg.addVector({Tab.translation().x, Tab.translation().y}, Tab(v_bhat), std::string{"brown"});
    mysvg.addVector({0, 0}, v_a, std::string{"purple"});
    mysvg.addVector({Tac.translation().x, Tac.translation().y}, Tac(v_c), std::string{"orange"});

    std::ofstream ofs ("/tmp/frames.svg", std::ofstream::out);

    ofs << mysvg.getOutput().rdbuf();

    ofs.close();

    return (0);
}