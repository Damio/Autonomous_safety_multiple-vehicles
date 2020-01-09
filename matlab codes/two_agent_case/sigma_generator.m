function [ sig ] = sigma_generator(r, r_o, rho, rho_e, rho_o, f)

%   This function takes in the location of the object and the obstacles, and gives out
%   a sigma for each obstacle

%define regions around the object and the obstacle
beta = rho_o^2 - ((norm(r-r_o))^2);
beta_z = (-2*rho_o*(rho+rho_e)) - ((rho+rho_e)^2);
beta_f = (-2*rho_o*(rho+rho_e+f)) - ((rho+rho_e+f)^2); 

a = 2/((beta_z - beta_f)^3);
b = - (3*(beta_z + beta_f)/(beta_z - beta_f)^3);
c = (6*beta_z*beta_f)/((beta_z - beta_f)^3);
d = (beta_z^2)*(beta_z-(3*beta_f))/((beta_z - beta_f)^3);


if beta < beta_f
    sig = 1;
elseif beta_f <= beta && beta_z >= beta
    sig = (a * (beta^3)) + (b * (beta^2)) + (c * beta) + d;    
else
    sig = 0;


end

end



    
