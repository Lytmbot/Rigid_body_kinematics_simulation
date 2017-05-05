function [ LEFT, RIGHT ] = build_springs(x)
% Build Springs, based of compression/extentions from current x(t)

% IN:
%   - only need current x(t), in {1}
%   - get system parameters from ParameterSheet()

% OUT:
%   -LEFT, points that make up left spring in {1}
%   -RIGHT, points that make up right spring in {1}

h   = ParametersSheet('h');
s_w = ParametersSheet('s_w');
s_l = ParametersSheet('s_l');
lo  = ParametersSheet('lo');

L = abs((-lo-x+s_l/2)/7); % left spacing
R = abs(( lo-x-s_l/2)/7); % right spacing

%% LEFT SPRING
LEFT = [ x-s_l/2 , (x-s_l/2)-L,   (x-s_l/2)-2*L,   (x-s_l/2)-3*L,   (x-s_l/2)-4*L,   (x-s_l/2)-5*L,   (x-s_l/2)-6*L, (x-s_l/2)-7*L ;
               h ,   h, h+s_w/2, h-s_w/2, h+s_w/2, h-s_w/2, h+s_w/2, h   ];


%% RIGHT SPRING
RIGHT = [ x+s_l/2 , (x+s_l/2)+R,  (x+s_l/2)+2*R,    (x+s_l/2)+3*R,   (x+s_l/2)+4*R,   (x+s_l/2)+5*R,   (x+s_l/2)+6*R, (x+s_l/2)+7*R ;
                h ,   h, h+s_w/2, h-s_w/2, h+s_w/2, h-s_w/2, h+s_w/2, h   ];


end




