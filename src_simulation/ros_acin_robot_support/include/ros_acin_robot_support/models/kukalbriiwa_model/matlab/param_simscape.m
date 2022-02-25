%% Robot parameters from Simscape in Fachvertiefungs coordinate system
%--------------------------------------------------------------------------
% This file contains the iiwa parameters, which is read from
% Simscape model without changes.
%  author:     Moien.R, TW
%  creation    date:07.05.2020
%---------------------------------
%% World coordinaten system
param_robot.R_0=eye(3);
param_robot.p_0=[0;0;0];

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
param_robot.m1=4;
param_robot.m2=4;
param_robot.m3=3;
param_robot.m4=2.7;
param_robot.m5=1.7;
param_robot.m6=1.8;
param_robot.m7=0.3;

%% Link center of gravity in inertial frame
param_robot.sp1x=0;   
param_robot.sp1y=-0.03;
param_robot.sp1z=0.12;

param_robot.sp2x=0.0003;
param_robot.sp2y=0.059;
param_robot.sp2z=0.042;

param_robot.sp3x=0;
param_robot.sp3y=0.03;
param_robot.sp3z=0.13;

param_robot.sp4x=0;
param_robot.sp4y=0.067;
param_robot.sp4z=0.034;

param_robot.sp5x=0.0001;
param_robot.sp5y=0.021;
param_robot.sp5z=0.076;

param_robot.sp6x=0;
param_robot.sp6y=0.0005999999999999999;
param_robot.sp6z=0.0004;

param_robot.sp7x=0;
param_robot.sp7y=0;
param_robot.sp7z=0.02;

%% Link moments of inertia    
param_robot.I1xx=0.1;
param_robot.I1yy=0.09000000000000001;
param_robot.I1zz=0.02;
param_robot.I1xy=0;
param_robot.I1yx=0;
param_robot.I1xz=0;
param_robot.I1zx=0;
param_robot.I1yz=0;
param_robot.I1zy=0;

param_robot.I2xx=0.05;
param_robot.I2yy=0.018;
param_robot.I2zz=0.044;
param_robot.I2xy=0;
param_robot.I2yx=0;
param_robot.I2xz=0;
param_robot.I2zx=0;
param_robot.I2yz=0;
param_robot.I2zy=0;

param_robot.I3xx=0.08000000000000002;
param_robot.I3yy=0.075;
param_robot.I3zz=0.009999999999999998;
param_robot.I3xy=0;
param_robot.I3yx=0;
param_robot.I3xz=0;
param_robot.I3zx=0;
param_robot.I3yz=0;
param_robot.I3zy=0;

param_robot.I4xx=0.03;
param_robot.I4yy=0.01;
param_robot.I4zz=0.029;
param_robot.I4xy=0;
param_robot.I4yx=0;
param_robot.I4xz=0;
param_robot.I4zx=0;
param_robot.I4yz=0;
param_robot.I4zy=0;

param_robot.I5xx=0.02;
param_robot.I5yy=0.018;
param_robot.I5zz=0.004999999999999999;
param_robot.I5xy=0;
param_robot.I5yx=0;
param_robot.I5xz=0;
param_robot.I5zx=0;
param_robot.I5yz=0;
param_robot.I5zy=0;

param_robot.I6xx=0.005;
param_robot.I6yy=0.0036;
param_robot.I6zz=0.0047;
param_robot.I6xy=0;
param_robot.I6yx=0;
param_robot.I6xz=0;
param_robot.I6zx=0;
param_robot.I6yz=0;
param_robot.I6zy=0;

param_robot.I7xx=0.001;
param_robot.I7yy=0.001;
param_robot.I7zz=0.001;
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
