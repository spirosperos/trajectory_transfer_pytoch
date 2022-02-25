#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <cmath>
#include <array>
#ifndef M_PI
    #define M_PI (3.1415926535897931)
#endif
struct RobotParams
{

    double g = 9.81;

    // this specifies the rotation around the Y-Axis of the gravitation vector
    double alpha_base = 0;
    double beta_base = 0;
    double gamma_base = 0;

    double dw0x = 0;
    double dw0y = 0;
    double dw0z = 0;

    const double d1 = 0.1575;
    const double d2 = 0.2025;
    const double d3 = 0.2375;
    const double d4 = 0.1825;
    const double d5 = 0.2175;
    const double d6 = 0.1825;
    const double d7 = 0.081;
    const double d8 = 0.071;

    //const double m0 = 5;
    const double m1 = 6.495;
    const double m2 = 8.807;
    const double m3 = 2.8;
    const double m4 = 5.283;
    const double m5 = 1.889;
    const double m6 = 2.32;
    const double m7_ = 0.863; //default not changeable
    double m7 = m7_;

/*
    const double I0xx = 0.05;
    const double I0xy = 0;
    const double I0xz = 0;
    const double I0yy = 0.06;
    const double I0yz = 0;
    const double I0zz = 0.03;
*/

    const double I1xx = 0.0690761441316632;
    const double I1xy = 0;
    const double I1xz = 0;
    const double I1yy = 0.0711085586914514;
    const double I1yz = 0;
    const double I1zz = 0.0200625854402119;

    const double I2xx = 0.0824701683534692;
    const double I2xy = 0;
    const double I2xz = 0;
    const double I2yy = 0.0164110968055191;
    const double I2yz = 0;
    const double I2zz = 0.0873510892661361;

    const double I3xx = 0.0230152941318817;
    const double I3xy = 0;
    const double I3xz = 0;
    const double I3yy = 0.0228661554143474;
    const double I3yz = 0;
    const double I3zz = 0.0554482283748983;

    const double I4xx = 0.0472789668341968;
    const double I4xy = 0;
    const double I4xz = 0;
    const double I4yy = 0.00964726804146909;
    const double I4yz = 0;
    const double I4zz = 0.0466529007761679;

    const double I5xx = 0.0138359953358589;
    const double I5xy = 0;
    const double I5xz = 0;
    const double I5yy = 0.0116859337751969;
    const double I5yz = 0;
    const double I5zz = 0.00571863785412578;

    const double I6xx = 0.00732758787216765;
    const double I6xy = 0;
    const double I6xz = 0;
    const double I6yy = 0.00477633755783711;
    const double I6yz = 0;
    const double I6zz = 0.00606797638550296;

    const double I7xx = 0.000884;
    const double I7xy = 0;
    const double I7xz = 0;
    const double I7yy = 0.000888;
    const double I7yz = 0;
    const double I7zz = 0.001105;

/*
    const double sp0x = -0.1;
    const double sp0y = 0;
    const double sp0z = 0.07;
*/

    const double sp1x = 0;
    const double sp1y = -0.01439;
    const double sp1z = 0.102503;

    const double sp2x = 1.2e-05;
    const double sp2y = 0.06472;
    const double sp2z = 0.004804;

    const double sp3x = -2.08e-04;
    const double sp3y = 0.01601;
    const double sp3z = 0.087283;

    const double sp4x = -2.03e-04;
    const double sp4y = 0.098639;
    const double sp4z = 0.011478;

    const double sp5x = 5.7e-05;
    const double sp5y = 0.014468;
    const double sp5z = 0.062938;

    const double sp6x = 3.63e-04;
    const double sp6y = 0.016982;
    const double sp6z = -0.019731;
    const double sp7x_ = 0.001904;
    const double sp7y_ = -9.3e-05;
    const double sp7z_ = 0.018862;

    double sp7x = sp7x_;
    double sp7y = sp7y_;
    double sp7z = sp7z_;
    const double K__1=60000;
    const double K__2=60000;
    const double K__3=40000;
    const double K__4=40000;
    const double K__5=40000;
    const double K__6=20000;
    const double K__7=20000;
    //friction
    const double beta = 0;
  //Gear ratios for each link
     const double n__1=160;
     const double n__2=160;
     const double n__3=160;
     const double n__4=160;
     const double n__5=100;
     const double n__6=160;
     const double n__7=160;
     //Rotor moments of inertia
     //A__i is taken as TorqueOfInertia (M00i.xml). As a consequence, the rotor
     //by itself has the same inertia around every axis.
     //B__i is taken as the sum of TorqueOfInertia (M00i.xml) and
     //ExtTorqueOfInertia (Ai.xml)
     //Only the component B__i is multiplied by the gear ratio n__i^2, since the
     //rotation of the rotor is amplified by n__i: theta__i=q__i*n__i and the
     //inertia is therefor amplified by n__i^2. The other axes A__i are not
     //amplified and therefore stay small.
     const double I__r1xx=0.000185;
     const double I__r1yy=I__r1xx;
     const double I__r1zz=0.000185+0.000238166953125;
     const double I__r2xx=0.000185;
     const double I__r2yy=I__r2xx;
     const double I__r2zz=0.000185+0.000238166953125;
     const double I__r3xx=0.000129;
     const double I__r3yy=I__r3xx;
     const double I__r3zz=0.000129+6.20384375e-005;
     const double I__r4xx=0.000129;
     const double I__r4yy=I__r4xx;
     const double I__r4zz=0.000129+6.20384375e-005;
     const double I__r5xx=7.5e-05;
     const double I__r5yy=I__r5xx;
     const double I__r5zz=7.5e-05+7.20968e-005;
     const double I__r6xx=1.5e-05;
     const double I__r6yy=I__r6xx;
     const double I__r6zz=1.5e-05+3.51125e-006;
     const double I__r7xx=1.5e-05;
     const double I__r7yy=I__r7xx;
     const double I__r7zz = 1.5e-05+3.51125e-006;
   //Moments of inertia of the rotors
     const double B[7] = {n__1*n__1*I__r1zz, n__2*n__2*I__r2zz,n__3*n__3*I__r3zz,n__4*n__4*I__r4zz,n__5*n__5*I__r5zz,n__6*n__6*I__r6zz, n__7*n__7*I__r7zz};
    const double deg_to_rad = M_PI / 180.0;
    const std::array<double, 7> tau_max = {320, 320, 176, 176, 110, 40, 40};
    const std::array<double, 7> Tau_lb = {-320, -320, -176, -176, -110, -40, -40};
    const std::array<double, 7> Tau_ub = {320, 320, 176, 176, 110, 40, 40};
    const std::array<double, 7> q_limit_upper = {170 * deg_to_rad, 120 * deg_to_rad, 170 * deg_to_rad, 120 * deg_to_rad, 170 * deg_to_rad, 120 * deg_to_rad, 175 * deg_to_rad};
    const std::array<double, 7> q_limit_lower = {-170 * deg_to_rad, -120 * deg_to_rad, -170 * deg_to_rad, -120 * deg_to_rad, -170 * deg_to_rad, -120 * deg_to_rad, -175 * deg_to_rad};
};

#endif // ROBOT_PARAMS_H
