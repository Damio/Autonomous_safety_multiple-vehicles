function [r,tetha ] = get_pos(r,tetha, x_dot, y_dot, tetha_dot,count,dt,rho,agent)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    

%calculate new position
    x = x_dot * dt;
    y = y_dot * dt;
    new_tetha = tetha_dot*dt;
        
    
    %update r & tetha
    r =  r + [x;y];   
    tetha = wrapToPi(tetha+new_tetha);
    
    
    
    %plot for vehicle 
    xlim('auto')
    ylim('auto')
    if agent == 'A'
        %plot(r(1),r(2),'k.')
        
        if mod(count,100)== 0
            r_centers = r.';
            viscircles(r_centers,rho,'Color','k');
        else
        end
    else   
        %plot(r(1),r(2),'g.')
        
        if mod(count,100)== 0
            r_centers = r.';
            viscircles(r_centers,rho,'Color','g');
        else
        end
    end
    
    
end

