%% iiwa Parameters from KUKA in Fachvertiefung coordinate system
%--------------------------------------------------------------------------
% This file contains the already identified iiwa parameters from KUKA,
% converted in Fachvertiefung coordinate system
%  author:     Moien.R, TW
%  creation    date:07.05.2020
%---------------------------------
%% World-coordinate system
param_robot.R_0=eye(3);
param_robot.p_0=[0;0;0];

% Base Rotation around Y for handing / Sally mount
param_robot.base_rotation_y = 0 * pi / 180;
R_base_y = [cos(param_robot.base_rotation_y), 0, sin(param_robot.base_rotation_y); 0, 1, 0; -sin(param_robot.base_rotation_y), 0, cos(param_robot.base_rotation_y)];
param_robot.R_0 = param_robot.R_0 * R_base_y;

% Roboter stands
param_robot.g=9.81;

%% Length of links
param_robot.d1          = 0.1575;
param_robot.d2          = 0.2025;
param_robot.d3          = 0.2375;
param_robot.d4          = 0.1825;
param_robot.d5          = 0.2175;
param_robot.d6          = 0.1825;
param_robot.d7          = 0.081;
param_robot.d8          = 0.071;

%% Mass of links
param_robot.m1=6.495;
param_robot.m2=8.807;
param_robot.m3=2.8;
param_robot.m4=5.283;
param_robot.m5=1.889;
param_robot.m6=2.32;
param_robot.m7=0.863;

%% Link center of gravity in inertial frame
param_robot.sp1x=0;
param_robot.sp1y=-0.014390000000000;
param_robot.sp1z=0.107503000000000;

param_robot.sp2x=1.200000000000000e-05;
param_robot.sp2y=0.064720000000000;
param_robot.sp2z=0.004804000000000;

param_robot.sp3x=-2.080000000000000e-04;
param_robot.sp3y=0.016010000000000;
param_robot.sp3z=0.0822830000000000;

param_robot.sp4x=-2.030000000000000e-04;
param_robot.sp4y=0.098639000000000;
param_robot.sp4z=0.011478000000000;

param_robot.sp5x=5.700000000000000e-05;
param_robot.sp5y=0.014468000000000;
param_robot.sp5z=0.0579380000000000;

param_robot.sp6x=3.630000000000000e-04;
param_robot.sp6y=0.0169820000000000;
param_robot.sp6z=-0.0197310000000000;

param_robot.sp7x=0.001904000000000;
param_robot.sp7y=-9.300000000000000e-05;
param_robot.sp7z=0.0174619999999999;

%% Link moments of inertia
param_robot.I1xx=0.0690761441316632;
param_robot.I1yy=0.0711085586914514;
param_robot.I1zz=0.0200625854402119;
param_robot.I1xy=0;
param_robot.I1yx=0;
param_robot.I1xz=0;
param_robot.I1zx=0;
param_robot.I1yz=0;
param_robot.I1zy=0;

param_robot.I2xx=0.0824701683534692;
param_robot.I2yy=0.0164110968055191;
param_robot.I2zz=0.0873510892661361;
param_robot.I2xy=0;
param_robot.I2yx=0;
param_robot.I2xz=0;
param_robot.I2zx=0;
param_robot.I2yz=0;
param_robot.I2zy=0;

param_robot.I3xx=0.0230152941318817;
param_robot.I3yy=0.0228661554143474;
param_robot.I3zz=0.0554482283748983;
param_robot.I3xy=0;
param_robot.I3yx=0;
param_robot.I3xz=0;
param_robot.I3zx=0;
param_robot.I3yz=0;
param_robot.I3zy=0;

param_robot.I4xx=0.0472789668341968;
param_robot.I4yy=0.00964726804146909;
param_robot.I4zz=0.0466529007761679;
param_robot.I4xy=0;
param_robot.I4yx=0;
param_robot.I4xz=0;
param_robot.I4zx=0;
param_robot.I4yz=0;
param_robot.I4zy=0;

param_robot.I5xx=0.0138359953358589;
param_robot.I5yy=0.0116859337751969;
param_robot.I5zz=0.00571863785412578;
param_robot.I5xy=0;
param_robot.I5yx=0;
param_robot.I5xz=0;
param_robot.I5zx=0;
param_robot.I5yz=0;
param_robot.I5zy=0;

param_robot.I6xx=0.00732758787216765;
param_robot.I6yy=0.00477633755783711;
param_robot.I6zz=0.00606797638550296;
param_robot.I6xy=0;
param_robot.I6yx=0;
param_robot.I6xz=0;
param_robot.I6zx=0;
param_robot.I6yz=0;
param_robot.I6zy=0;

param_robot.I7xx=0.000884;
param_robot.I7yy=0.000888;
param_robot.I7zz=0.001105;
param_robot.I7xy=0;
param_robot.I7yx=0;
param_robot.I7xz=0;
param_robot.I7zx=0;
param_robot.I7yz=0;
param_robot.I7zy=0;

%% Limits
param_robot.tau_max=diag([320.00,320.00,176.00,176.00,110.00,40.00,40.00]);
param_robot.q_limit_upper=[170;120;170;120;170;120;175]*pi/180;
param_robot.q_limit_lower=-param_robot.q_limit_upper;

