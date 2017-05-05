function [value,isterminal,direction] = collision_detection(~,X)

    collision_point = abs(ParametersSheet('lo') - ParametersSheet('s_l')/2);  
        
    %the conditions when hte collision occurs
    value = abs(X(1))-collision_point;    % measure distance from shuttle to end of rail
    
    if value >= 0 
       disp('collision detection at state:')
       X
    end
    
    isterminal = 1;     % switch to stop the simulation
    direction = 1;      % switch direction of detection e.g. for an object that thrown up and then falls
                        % dont detect an increase in value as an event 0 is doents matter?    

end