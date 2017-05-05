function [ X_plus ] = MODEL_COLLISION( X_minus )

%% MODEL THE COLLISION OF THE SHUTTLE WITH THE END OF THE RAIL

% INPUT:
%   X(4*1) matrix of initial conditions at collision

%OUTPUT:
%   X_plus(4*1) matrix of state after collision, only x_dot and theta_dot
%   are effected in any meaningful way
    
m1 = ParametersSheet( 'm1' );
m2 = ParametersSheet( 'm2' );
h = ParametersSheet( 'h' );
k = ParametersSheet( 'k' );
r = ParametersSheet( 'r' );
lo = ParametersSheet( 'lo' );
e = ParametersSheet( 'e' );


%% SWAP VELOCITY METHOD
X_plus(1) = X_minus(1)*0.99;  % x plus tiny change so doesnt trigger collision_detection
X_plus(2) = X_minus(2);      % theta plus no change
X_plus(3) = X_minus(3)*(-1*e);   % x dot plus shuttle linear velocity
%X_plus(4) = X_minus(4);    % theta dot plus angulare velocity

% NEWTON EULER (lyles) errors...
%const = h + ( m1*r^2*cos(atan(lo/h)) )/( 2*m2*sqrt(lo^2 + h^2) );
%X_plus(4) = ( X_minus(4)*(const) - 2*X_minus(3) )/(const  );

%% USE COLLISION MODEL (Rinas)
const = (h*(e+1))/( (m1*r^2)/(m2*2) + h^2 + lo^2 );
X_plus(4) = X_minus(4) - const*X_minus(3);


end

