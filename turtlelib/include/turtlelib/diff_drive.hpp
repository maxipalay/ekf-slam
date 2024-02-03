#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics for a differential drive robot.

#include "turtlelib/se2d.hpp"
#include <tuple>
#include <exception>

namespace turtlelib {
    
    /// \brief A class that represents a diff drive robot's kinematics
    class DiffDrive {
        private:
            // args
            double wheelTrack;
            double wheelRadius;
            double phiLeft{};
            double phiRight{};
            Transform2D config{};
            
            struct Pseudoinv {double p11, p12, p13, p21, p22, p23;};

            Pseudoinv leftPinv {};

            void InitializePseudoInv();

        public:

            /// \brief create an instance of a DiffDrive robot model
            /// \param track - distance between driving wheel centers
            /// \param radius - radius of wheels (meters) 
            DiffDrive(double track, double radius);

            /// \brief perform forward kinematics given the updated wheel angles  
            /// \param radLeft - angle of left wheel (radians)
            /// \param radRight - angle of right wheel (radians)
            /// \return the updated transform
            Transform2D FKin(double radLeft, double radRight);

            /// \brief perform inverse kinematics given a twist
            /// \param twist - the desired twist
            /// \return tuple with wheel velocities to achieve the twist
            std::tuple <double, double> IKin(Twist2D twist);

            /// \brief get the current configuration
            /// \return the current Transform2D
            Transform2D getConfig();

    };

}
#endif