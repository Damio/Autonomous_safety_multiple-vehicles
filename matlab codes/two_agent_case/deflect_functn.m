function [ Fo_n, lambda ] = deflect_functn( r, ro, r_g)
%this function takes in the location of the object with respect to the
%obstacles and the defined regions z & f that define minimum distances

delta_r = r - ro;
x = delta_r(1);
y = delta_r(2);



xo = ro(1)- r_g(1);
yo = ro(2)- r_g(2);

%calculate phi for each obstacle
phi = (atan2(-yo, -xo) + (pi));
P = [cos(phi), sin(phi)];
Px = P(1);
Py = P(2);

%this section ensures that the fields around the obstacle have the right
%orientation

%distance between r_o & r_g
% d = norm(ro - r_g);
% %extracting the points for calculations
% x0 = ro(1);
% x1 = r_g(1);
% y0 = ro(2);
% y1 = r_g(2);
% 
% %defining the points on Vi1 & Vi2 that lie on beta_z & beta_o.
% A = get_line(-rho_o,d,x0,x1,y0,y1);
% B = get_line(-rho_z,d,x0,x1,y0,y1);
% C = get_line(rho_o,d,x0,x1,y0,y1);
% D = get_line(rho_z,d,x0,x1,y0,y1);
%(norm(A-r) + norm(r-B) == norm(A-B)) || (norm(C-r) + norm(r-D) == norm(C-D))

%condition 1
if (P * delta_r) < 0
    lambda = 0;
    Fo_x = ((lambda-1)*Px*(x^2)) + (lambda*Py*x*y) - (Px*(y^2));
    Fo_y = ((lambda-1)*Py*(y^2)) + (lambda*Px*x*y) - (Py*(x^2));

    Fo = [Fo_x; Fo_y];

    Fo_n = Fo / (norm(Fo));
    
%condition 2 
elseif ((P * delta_r)) >= 0 && (((Py*x) - (Px*y))~= 0)

    lambda = 1;
    Fo_x = ((lambda-1)*Px*(x.^2)) + (lambda*Py*x.*y) - (Px*(y.^2));
    Fo_y = ((lambda-1)*Py*(y.^2)) + (lambda*Px*x.*y) - (Py*(x.^2));

    Fo = [Fo_x; Fo_y];

    Fo_n = Fo / (norm(Fo));
% %condition 3    
% elseif (P * delta_r) >= 0 && ((Py*x) - (Px*y))~= 0
%     Fo_n = [0; 0];
%     lambda = 0;


%condition 3 [((transpose{P} * delta_ri) >= 0) && ((Py*x) - (Px*y))== 0]
else    

    Fo_n = [0; 0];
    lambda = 1;
end
%extract the new x and y
% ttt = ((Py*x) - (Px*y))
% bbb = ((Py*r(1)) - (Px*r(2)))

end


%function that calculates points from this expression
% function [Z] = get_line(dt,d,x0,x1,y0,y1)
% t = dt/d;
% Xt = ((1-t)*x0) + (t*x1);
% Yt = ((1-t)*y0) + (t*y1);
% Z = [Xt;Yt];
% end
