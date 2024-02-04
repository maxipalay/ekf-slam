#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib {

    DiffDrive::DiffDrive(double track, double radius){
        wheelTrack = track;
        wheelRadius = radius;
        InitializePseudoInv();
    }

    void DiffDrive::InitializePseudoInv(){
        // Moore-Penrose left-pseudoinverse: ((A*A)^(-1))A*
        leftPinv.p11 = -wheelTrack/wheelRadius;
        leftPinv.p12 = 1.0/wheelRadius;
        leftPinv.p13 = 0.0;
        leftPinv.p21 = wheelTrack/wheelRadius;
        leftPinv.p22 = 1.0/wheelRadius;
        leftPinv.p23 = 0.0;
    }

    Transform2D DiffDrive::FKin(double radLeft, double radRight){
        // get angle differences
        auto angleDiffLeft = radLeft - phiLeft;
        auto angleDiffRight = radRight - phiRight;
        angleDiffRight *= -1.0;
        // body twist: Lynch, Park - Modern Robotics 13.4
        auto bodyTwist = Twist2D{wheelRadius/wheelTrack*(-angleDiffLeft+angleDiffRight), 
                                 wheelRadius/2.0*(angleDiffLeft+angleDiffRight), 0.0};
        // integrate body twist to get Tbb'
        auto tf = integrate_twist(bodyTwist);
        // update config Twb*Tbb' = Twb'
        config *= tf;
        // track wheel angles
        phiLeft = normalize_angle(radLeft);
        phiRight = normalize_angle(radRight);
        return config;
    }

    wheelSpeeds DiffDrive::IKin(Twist2D twist){
        if (!almost_equal(twist.y, 0.0)){
            throw std::logic_error("wheels would slip for input twist!");
        }
        auto velLeft = leftPinv.p11*twist.omega+leftPinv.p12*twist.x;
        auto velRight = leftPinv.p21*twist.omega+leftPinv.p22*twist.x;
        // negative sign precedes due to the orientation of the wheel frame
        return wheelSpeeds{velLeft,-velRight};
    }

    Transform2D DiffDrive::getConfig(){
        return config;
    }




}