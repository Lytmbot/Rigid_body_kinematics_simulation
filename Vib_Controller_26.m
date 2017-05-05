function [ t,x] = Vib_Controller( )
close all; 
clear all;

%% DESCRIPTION
% runs the simulation based of paramaters set in ParametersSheet.m
%
% INPUT: nil
% OUTPUT: every graphs you ever needed
%-------------------------------------------------------------------------%

%% SET UP
r = ParametersSheet( 'r' );
lo = ParametersSheet( 'lo' );
s_l = ParametersSheet( 's_l' );
s_w = ParametersSheet( 's_w' );
h = ParametersSheet( 'h' );
STEP = ParametersSheet( 'STEP' );
T0     = ParametersSheet( 'T0' );
TF     = ParametersSheet( 'TF' );
t = zeros(1,1);
X = zeros(1,4);
options     = ParametersSheet( 'options' );
x_initial     = ParametersSheet( 'x_initial' );
make_video = 0; % yes = 1, no = 0;

%% VIDEO SET UP
writerObj = VideoWriter('simulation.mp4', 'MPEG-4'); % Name it.
writerObj.FrameRate = 100; % How many frames per second. 1/dt = 100
open(writerObj); 

%% FIGURE HANDLES
figure()
fig_states = axes;
hold(fig_states,'off');

figure()
fig_sym = axes;
hold(fig_sym,'off');
axis(fig_sym,[ -2, 2, -2, 2]);

figure()
fig_spring = axes;
hold(fig_spring,'off');
axis(fig_spring,[ -(3/2)*lo, (3/2)*lo, -(3/2)*s_w, (3/2)*s_w]);

%% EVELUATION LOOP
while T0 < TF

    x_dot = @(t,x) SystemDynamics(t,x);
    [T_nominal,X_nominal] = ode45(x_dot,[T0:STEP:(TF+STEP)],x_initial,options);
    t = vertcat(t,T_nominal);
    X = vertcat(X,X_nominal);
    
    %% MODEL COLLISION    
    x_initial = MODEL_COLLISION(X(end,:));   
    T0 = t(end);
    
end
% --------------------
% X is state variables
% X(1) = x(t)
% X(2) = O(t)  -> 'theta'
% X(3) = x_dot(t)
% X(4) = O_dot(t)

X = transpose(X); % its a pancake

%% GET P values
% you will need to change this manually
P = log(t+1)*0.2;

%% PLOT EVERYTHING in X agains t
disp('simulation finished')
plot(fig_states, transpose(t),X, transpose(t), P) 
legend(fig_states, 'X(1) = ^1x(t)','X(2) = theta','X(3) = ^1x(t)_{dot}','X(4) = theta_{dot}', 'P = 1.5*sin(t)t')

roa_in_0 = [ X(1,:).*cos(X(2,:)) - h.*sin(X(2,:)) ; % convert X(1) = x(t)
             X(1,:).*sin(X(2,:)) + h.*cos(X(2,:))]; % to frame {0} from {1}

%% SIMULATION LOOP based of X(t) = states         
for i = 1 : 1 : size(t,1)    
   
    theta_t = X(2,i); 
    x_t = X(1,i);
     
   %% shuttle
   shuttle_in1 = [x_t-s_l/2 , x_t-s_l/2 , x_t+s_l/2 , x_t+s_l/2 , x_t-s_l/2 ; 
                    h-s_w/2 ,   h+s_w/2 ,   h+s_w/2 ,   h-s_w/2 , h-s_w/2 ];    
   R01 = [ cos(theta_t),-sin(theta_t); % 2D rotation matrix
           sin(theta_t), cos(theta_t)];   
   shuttle_in0 = R01*shuttle_in1;   
   
   %% rail
   rail_in_1 = [       -lo,       -lo,        lo,        lo,       -lo ;
                 h-s_w/1.3, h+s_w/1.3, h+s_w/1.3, h-s_w/1.3, h-s_w/1.3];
   rail_in_0 = R01*rail_in_1;   
   
   %% springs 
   [left_in1, right_in1] = build_springs(x_t);
   left = R01*left_in1;
   right = R01*right_in1;
     
   %% disk
   a = 0:pi/25:2*pi;  
   disk_in_1 = [r*cos(a) ; r*sin(a)];
   disk_in_0 = R01*disk_in_1;
   
   %% plot frame 0
   pause(.01);
   plot(fig_sym, roa_in_0(1,1:i), roa_in_0(2,1:i), ':') ;               % plot m2's path
   hold(fig_sym,'on');
   axis(fig_sym,[ -1.7, 1.7, -1.7, 1.7]);
   plot(fig_sym, [roa_in_0(1,i),0], [roa_in_0(2,i),0], 'r');            % plot line from O to m2
   plot(fig_sym, shuttle_in0(1,:), shuttle_in0(2,:) , 'k');             % render shuttle
   plot(fig_sym, rail_in_0(1,:), rail_in_0(2,:) , 'k');                 % render rail
   plot(fig_sym, disk_in_0(1,1:49), disk_in_0(2,1:49) , 'k');           % render 49/50ths of the disk in black
   plot(fig_sym, disk_in_0(1,49:51), disk_in_0(2,49:51) , 'g');         % render remaining 2/50ths in green as refrence
   plot(fig_sym, left(1,:),left(2,:),'k',right(1,:),right(2,:),'k');    % render springs
   pbaspect(fig_sym,[1,1,1]);
   hold(fig_sym,'off');
   
   %% plot frame 1
   plot(fig_spring, shuttle_in1(1,:), shuttle_in1(2,:) , 'k');
   hold(fig_spring,'on');
   axis(fig_spring,[ -1.1*lo, 1.1*lo, h-4*s_w, h+4*s_w]);
   plot(fig_spring, rail_in_1(1,:), rail_in_1(2,:) , 'k');
   plot(fig_spring,left_in1(1,:),left_in1(2,:),'k',right_in1(1,:),right_in1(2,:),'k');
   
   txt_loc = [x_t, h+1.5*s_w]; 
   plot(fig_spring, [txt_loc(1),x_t], [txt_loc(2),h]); % makes velocity value appear above shuttle
   text(txt_loc(1), txt_loc(2), strcat('$^1\dot{r}_{OM_2}$=  ', num2str(round(X(3,i),4))),'Interpreter','latex');
   hold(fig_spring,'off');
   
   %% MAKE VIDEO
   if make_video
        frame = getframe(fig_sym);
        writeVideo(writerObj, frame);
   end
end


end

