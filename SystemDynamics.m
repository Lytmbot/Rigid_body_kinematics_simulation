function [ X_dot ] = SystemDynamics(t,X)
% eveluate x_dd O_dd for given state, form X_dot[]

% INPUT
%   - X, 1x8 vector with x -> O_dot for time t

% OUTPUT
%   -X_dot, 1x8 vector with x_dot -> O_ddot

m1 = ParametersSheet( 'm1' );
m2 = ParametersSheet( 'm2' );
h = ParametersSheet( 'h' );
k = ParametersSheet( 'k' );
r = ParametersSheet( 'r' );
lo = ParametersSheet( 'lo' );

P =  log(t+1)*0.2;

x = X(1);
x_d = X(3);
O_d = X(4);
 
x_dd = -(- 2*O_d^2*h^2*m2^2*x - 2*O_d^2*m2^2*x^3 - m1*O_d^2*m2*r^2*x + 4*x_d*O_d*h*m2^2*x + 4*k*h^2*m2*x - 2*P*h*m2 + 4*k*m2*x^3 + 2*k*m1*r^2*x)/(m2*(m1*r^2 + 2*m2*x^2));
%x_dd =  -(- 2*O_d^2*h^2*m2^2 - 2*O_d^2*m2^2*x - m1*O_d^2*m2*r^2 + 2*x_d*O_d*h*m2^2 + 4*k*h^2*m2*x + 4*k*m2*x^2 + 2*k*m1*r^2*x)/(m2*(m1*r^2 + 2*m2*x));
%O_dd =  -(2*(- h*m2*O_d^2 + m2*x_d*O_d + 2*h*k*x))/(m1*r^2 + 2*m2*x);
O_dd  =   (2*(h*m2*x*O_d^2 - 2*m2*x*x_d*O_d + P - 2*h*k*x))/(m1*r^2 + 2*m2*x^2);

 
%% UPDATE
X_dot = [x_d; O_d; x_dd; O_dd];
end
