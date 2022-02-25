% Parameter for the simscape visualiziation
%
% Autor: RL, TW, MR 
% Date of creation: 27.07.2020

%% Trace of end-effector
par.vis_point.pix = 6;                          % pixel size
par.vis_point.col = [0.4, 1, 0.2];              % colour
par.vis_point.opa = 1;                          % transparency
par.vis_point.delta = round(par.T/100/par.Ta);  % distance of time between points

%% Whitebord
par.x_wb = [0.8; 0; 0.5; 0; pi/2; 0];   % pose
par.Wb_col = [1 1 1];                   % colour
par.Wb_opa = 0.8;                       % transparency

%transformation of coordinaten system frome base (0) to whiteboard (w)
par.R_0w = eul2rotm(par.x_wb(4:6)');
par.T_0w = [par.R_0w, par.x_wb(1:3); 0,0,0,1];


%% Robot gravity
param_robot.g_vis=[0;0;param_robot.g];

%% iiwa_link_0
% Mass of link
param_robot.link__0.m__0= 5 ;
% Moments of Inertia 
param_robot.link__0.I__0=[0.05, 0.06, 0.03000000000000001];

%joint
param_robot.link__0.R__joint1           = eye(3);
param_robot.link__0.d__joint1           = [0, 0, param_robot.d1];
%inertia
param_robot.link__0.R__inertiaOrigin    = eye(3);
param_robot.link__0.d__inertiaOrigin    = [-0.1, 0, 0.07000000000000001];

%% iiwa_link_1

% Mass of link
param_robot.link__1.m__1= param_robot.m1;

% Moments of Inertia 
param_robot.link__1.I__1=[param_robot.I1xx, param_robot.I1yy, param_robot.I1zz];

%joint
param_robot.link__1.R__joint2           = getRotationMatrix(-pi,'z')*getRotationMatrix(pi/2,'x');
param_robot.link__1.d__joint2           = [ 0, 0, param_robot.d2];
%inertia
param_robot.link__1.R__inertiaOrigin    = eye(3);
param_robot.link__1.d__inertiaOrigin    = [param_robot.sp1x, param_robot.sp1y, param_robot.sp1z];
%% iiwa_link_2

% Mass of link
param_robot.link__2.m__2= param_robot.m2;

% Moments of Inertia 
param_robot.link__2.I__2=[param_robot.I2xx, param_robot.I2yy, param_robot.I2zz];

%joint
param_robot.link__2.R__joint3           = getRotationMatrix(-pi,'z')*getRotationMatrix(pi/2,'x');
param_robot.link__2.d__joint3           = [0, param_robot.d3, 0];
%inertia
param_robot.link__2.R__inertiaOrigin    = eye(3);
param_robot.link__2.d__inertiaOrigin    = [param_robot.sp2x, param_robot.sp2y, param_robot.sp2z];
%% iiwa_link_3

% Mass of link
param_robot.link__3.m__3= param_robot.m3;

% Moments of Inertia 
param_robot.link__3.I__3=[param_robot.I3xx, param_robot.I3yy, param_robot.I3zz];

%joint
param_robot.link__3.R__joint4           = getRotationMatrix(pi/2,'x');
param_robot.link__3.d__joint4           = [0, 0, param_robot.d4];
%inertia
param_robot.link__3.R__inertiaOrigin    = eye(3);
param_robot.link__3.d__inertiaOrigin    = [param_robot.sp3x, param_robot.sp3y, param_robot.sp3z];

%% iiwa_link_4

% Mass of link
param_robot.link__4.m__4= param_robot.m4;
% Moments of Inertia 
param_robot.link__4.I__4=[param_robot.I4xx, param_robot.I4yy, param_robot.I4zz];

%joint
param_robot.link__4.R__joint5           = getRotationMatrix(-pi,'z')*getRotationMatrix(pi/2,'x');
param_robot.link__4.d__joint5           = [0, param_robot.d5, 0];
%inertia
param_robot.link__4.R__inertiaOrigin    = eye(3);
param_robot.link__4.d__inertiaOrigin    = [param_robot.sp4x,  param_robot.sp4y, param_robot.sp4z];
%% iiwa_link_5

% Mass of link
param_robot.link__5.m__5= param_robot.m5;
% Moments of Inertia 
param_robot.link__5.I__5=[param_robot.I5xx, param_robot.I5yy, param_robot.I5zz];

%joint
param_robot.link__5.R__joint6           = getRotationMatrix(pi/2,'x');
param_robot.link__5.d__joint6           = [0, 0, param_robot.d6];
%inertia
param_robot.link__5.R__inertiaOrigin    = eye(3);
param_robot.link__5.d__inertiaOrigin    = [param_robot.sp5x, param_robot.sp5y, param_robot.sp5z];

%% iiwa_link_6

% Mass of link
param_robot.link__6.m__6= param_robot.m6;
% Moments of Inertia 
param_robot.link__6.I__6=[param_robot.I6xx, param_robot.I6yy, param_robot.I6zz];

%joint
param_robot.link__6.R__joint7           = getRotationMatrix(-pi,'z')*getRotationMatrix(pi/2,'x');
param_robot.link__6.d__joint7           = [0, param_robot.d7, 0];
%inertia
param_robot.link__6.R__inertiaOrigin    = eye(3);
param_robot.link__6.d__inertiaOrigin    = [param_robot.sp6x, param_robot.sp6y, param_robot.sp6z];

%% iiwa_link_7

% Mass of link
param_robot.link__7.m__7= param_robot.m7;
% Moments of Inertia 
param_robot.link__7.I__7=[param_robot.I7xx,param_robot.I7yy, param_robot.I7zz];

%joint
param_robot.link__7.R__ee               = eye(3);
param_robot.link__7.d__ee               = [0, 0, param_robot.d8];
%inertia
param_robot.link__7.R__inertiaOrigin    = eye(3);
param_robot.link__7.d__inertiaOrigin    = [param_robot.sp7x, param_robot.sp7y, param_robot.sp7z];

%% function to get rotation matrix
function R = getRotationMatrix(phi,dir)

switch dir
    case 'x'
        R = [1,0,0;cos(phi),0,-sin(phi);0,sin(phi),cos(phi)];
    case 'y'
        R = [cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)];
    case 'z'
        R = [cos(phi),-sin(phi),0;sin(phi),cos(phi),0;0,0,1];
    otherwise
        error('.')
end
return;
end

