function [ parm ] = ParametersSheet( name )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

switch name
%%%%% Dynamics System Parameters %%%%
    case {'k'}
        parm = 2;     %[kg];
    case {'m1'}
        parm = 1;      %[kg];
    case {'m2'}
        parm = 1;   %[kg]
    case {'r'}
        parm = 1.5;   %[m]
    case {'h'}
        parm = 1.2;   %[m]
    case {'s_w'}
        parm = .1;   %[m] shuttle width
    case {'s_l'}
        parm = .2;   %[m] shuttle length
    case {'lo'}
        parm = .5;   %[m] rail length
    case {'e'}
        parm = 1;   % coeeficent of restitution for collision model
     
        
%%%%% Control Parameters %%%%
    case {'Q'}
        parm = diag([10 1]);          
    case {'R'}
        parm = 5;          

%%%%% Simulation Parameters %%%%
    case {'T0'}
        parm = 0;
    case {'TF'}
        parm = 20;
    case {'STEP'}
        parm = 0.01;
    case('options')
        parm = odeset('Events', @collision_detection, 'RelTol',1e-12);
        % event detection for collision as well as motion simulation
        % the integration for simulation will stop if @collision_detection
        % is triggered
     case('gap')
        parm = 1.5; 
    case('Swingloop')
        parm = 10;
     case {'x_initial'}
        parm = [0;0;0;0];
        % [x0,theta0,x_d0,theta_d0]
    otherwise
          

end

