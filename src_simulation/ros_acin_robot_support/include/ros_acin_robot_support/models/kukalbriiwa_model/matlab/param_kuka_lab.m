%% Additional roboter parameters for lab
%--------------------------------------------------------------------------
% This file contains the additional iiwa parameters, that are needed for
% the laboratory.
%  author:     CH, TW
%  creation    date:07.05.2020
%---------------------------------

%Load rigid body parameter
param_kuka;

%Number of links
param_robot.n=7;

%% Axis limits
param_robot.q_limit_range = param_robot.q_limit_upper - param_robot.q_limit_lower;
param_robot.q_mean = zeros(7,1);

%% Additional Parameters
%Friction
param_robot.beta=0;
%Gear ratios for each link
param_robot.n__1=160;
param_robot.n__2=160;
param_robot.n__3=160;
param_robot.n__4=160;
param_robot.n__5=100;
param_robot.n__6=160;
param_robot.n__7=160;
param_robot.N=diag([param_robot.n__1,param_robot.n__2,param_robot.n__3,param_robot.n__4,param_robot.n__5,param_robot.n__6,param_robot.n__7]);
%Rotor moments of inertia
%A__i is taken as TorqueOfInertia (M00i.xml). As a consequence, the rotor
%by itself has the same inertia around every axis.
%B__i is taken as the sum of TorqueOfInertia (M00i.xml) and
%ExtTorqueOfInertia (Ai.xml)
%Only the component B__i is multiplied by the gear ratio n__i^2, since the
%rotation of the rotor is amplified by n__i: theta__i=q__i*n__i and the
%inertia is therefor amplified by n__i^2. The other axes A__i are not
%amplified and therefore stay small.
param_robot.I__r1xx=0.000185;
param_robot.I__r1yy=param_robot.I__r1xx;
param_robot.I__r1zz=0.000185+0.000238166953125;
param_robot.I__r2xx=0.000185;
param_robot.I__r2yy=param_robot.I__r2xx;
param_robot.I__r2zz=0.000185+0.000238166953125;
param_robot.I__r3xx=0.000129;
param_robot.I__r3yy=param_robot.I__r3xx;
param_robot.I__r3zz=0.000129+6.20384375e-005;
param_robot.I__r4xx=0.000129;
param_robot.I__r4yy=param_robot.I__r4xx;
param_robot.I__r4zz=0.000129+6.20384375e-005;
param_robot.I__r5xx=7.5e-05;
param_robot.I__r5yy=param_robot.I__r5xx;
param_robot.I__r5zz=7.5e-05+7.20968e-005;
param_robot.I__r6xx=1.5e-05;
param_robot.I__r6yy=param_robot.I__r6xx;
param_robot.I__r6zz=1.5e-05+3.51125e-006;
param_robot.I__r7xx=1.5e-05;
param_robot.I__r7yy=param_robot.I__r7xx;
param_robot.I__r7zz=1.5e-05+3.51125e-006;
%Moments of inertia of the rotors
param_robot.B=diag([param_robot.n__1^2*param_robot.I__r1zz,...
    param_robot.n__2^2*param_robot.I__r2zz,...
    param_robot.n__3^2*param_robot.I__r3zz,...
    param_robot.n__4^2*param_robot.I__r4zz,...
    param_robot.n__5^2*param_robot.I__r5zz,...
    param_robot.n__6^2*param_robot.I__r6zz,...
    param_robot.n__7^2*param_robot.I__r7zz]);
%Joint stiffness from 181023_Stiffness Measurement
param_robot.K__1=60000;
param_robot.K__2=60000;
param_robot.K__3=40000;
param_robot.K__4=40000;
param_robot.K__5=40000;
param_robot.K__6=20000;
param_robot.K__7=20000;
param_robot.K=diag([param_robot.K__1,param_robot.K__2,param_robot.K__3,param_robot.K__4,param_robot.K__5,param_robot.K__6,param_robot.K__7]);

%% Lab Parameters
param_controller.T_a=125e-6;

param_controller.singular_perturbation=struct();
param_controller.singular_perturbation.K_tau=[4;4;4;5;3;2.5;2.5];
param_controller.singular_perturbation.D_tau=[0.015;0.015;0.015;0.02;0.01;0.01;0.01];

param_initial_condition=struct();
param_initial_condition.q__0=zeros(param_robot.n,1);
param_initial_condition.theta__0=param_initial_condition.q__0+param_robot.K\gravitational_forces(param_initial_condition.q__0,param_robot);
param_initial_condition.q__p_0=zeros(7,1);
param_initial_condition.theta__p_0=zeros(7,1);

param_visualization.retardation=1;

param_sensor=struct();
param_sensor.T_a=param_controller.T_a;

param_nullspace=struct();
param_nullspace.q_limit_upper=[170;120;170;120;170;120;175;90]*pi/180;
param_nullspace.q_limit_lower=-param_nullspace.q_limit_upper;
param_nullspace.q_limit_range=10*ones(param_robot.n+1,1)*pi/180;
param_nullspace.q_limit_slope=1*ones(param_robot.n+1,1);
param_nullspace.q_limit_gain=50*ones(param_robot.n+1,1);
param_nullspace.barrierfunction=true;

%% Filter settings
filter_tau_T_1=1./(2*pi*[400;400;400;400;500;1000;1000]);
filter_tau_p_T_1=filter_tau_T_1;
filter_theta_p_T_1=filter_tau_T_1;

%Slow filters for rigid body controllers
filter_tau_slow_T_1=1./(2*pi*[200;200;200;200;200;200;200]);
filter_tau_p_slow_T_1=filter_tau_slow_T_1;
filter_theta_slow_T_1=filter_tau_slow_T_1;
filter_theta_p_slow_T_1=filter_tau_slow_T_1;

%Torque sensor offsets, measured and identified by Bischof on 25.06.2018
sensor_torque_offset=[0.53;-0.095;-0.189;0.07;-0.058;0.0025;-0.12];

%Filter for the second derivative of the path parameter theta
filter_theta_pp_T_1=1/(2*pi*10);
filter_q_pp_T_1=filter_tau_T_1;

%% Attach load to the endeffector
param_load=struct();
switch 0
    otherwise
        %No load
        param_load.m=0;
        param_load.s_x=0;
        param_load.s_y=0;
        param_load.s_z=0;
end
%Integrate the load into link 7
tmp=([param_robot.sp7x;param_robot.sp7y;param_robot.sp7z]*param_robot.m7...
    +[param_load.s_x;param_load.s_y;param_load.s_z]*param_load.m)/(param_robot.m7+param_load.m);
param_robot.sp7x=tmp(1);
param_robot.sp7y=tmp(2);
param_robot.sp7z=tmp(3);
param_robot.m7=param_robot.m7+param_load.m;
clear('tmp');
