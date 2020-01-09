%clear previous data
clc
clear global

%define time parameters & time step
dt = .25; % time step for integration (seconds)
T = 10000*dt; % measurement time step
%linear and angular velocity gains
k_u = 0.015;
k_w = 2.5;

%for moving case
v = VideoWriter('path3.avi');
open(v);
%simple iteration counter
count = 0;

numberOfObstacles = 1; %no. of obstacles

%vehicle radius
rho = 0.03; %predetermined
%obstacle radius
rho_o = 0.03; %predetermined too
%minimum distance between vehicle and obstacle
rho_e = 0.01; %predetermined as well
f = 0.01;

%obstacle plot parameters
min_obs_dist = 0.02; %distance between obstacles
rho_z = rho_o + rho + rho_e; %Region Z radius
rho_f = rho_z + f; %Region F radius

%______________________________________________________
%call test obstacle to generate random obstacles if you want
%obstacles = test_obs(12, rho_o);
%___________________________________________________________
%use mouse to select start and end positions
figure;
xlim([-.7 0]);
ylim([-.4 .2]);
%define start and end locations and start orientation
[startx,starty] = ginput(1);
r_start = [startx;starty];
r = r_start; % r is the location of the object at every time, initialized at starting point
plot (r(1),r(2), 'bx','MarkerSize', 10,'LineWidth',2);
text((r(1)+0.01),(r(2)+0.005),'start','FontWeight','bold')
hold on
%goal position
xlim([-.7 0]);
ylim([-.4 .2]);
[rgx,rgy] = ginput(1);
r_g = [rgx;rgy];
plot (r_g(1),r_g(2), 'rx','MarkerSize', 10,'LineWidth',2);
text((r_g(1)+0.01),(r_g(2)+0.005),'goal','FontWeight','bold')
hold on
drawnow() 

%this part gets the orientation of object wrt the goal 
rr = r - r_g;
tetha = 0;%(atan2(r(2),r(1)));

%obstacle plot
obstacles = zeros([2,numberOfObstacles]);
for i = 1: numberOfObstacles
    %define the window scale
    xlim([-.7 0]);
    ylim([-.4 .2]);
    [obs_X,obs_Y] = ginput(1);
    centers = [obs_X, obs_Y];
    plot(obs_X,obs_Y,'r.');
    %text(obs_X,obs_Y,num2str(i),'Color','r')
    obstacles(:,i) = centers;
    axis square
    viscircles(centers,rho_o,'Color','b');
    %viscircles(centers,rho_z,'LineStyle',':','Color','r','LineWidth',.5);
    viscircles(centers,rho_z,'Color','r','LineWidth',.5);
    viscircles(centers,rho_f,'LineStyle',':','Color','r','LineWidth',1);
    
    %hold on
end

hold off

disp("OBSTACLES COMPLETE");


%_________________________________________________________________________________
%the motion simulation starts here 
for tau = dt : dt : T
    % initialize neccessary variables
    sig_array = zeros([numberOfObstacles,1]);
    sig_finold = 1;
    Fo_starold = [0;0];
    lam_array = zeros([numberOfObstacles,1]);
    
    %gets sigma for all  obstacles with respect to r and store in array
    for i = 1 : numberOfObstacles
        r_o = obstacles(:,i);
        sig = sigma_generator(r, r_o, rho, rho_e, rho_o, f);
        sig_array(i) = sig;
        sig_final = sig_finold*sig;
        sig_finold = sig_final;
        centr = transpose(r_o);
        %update the visuals for the moving upstacles
        viscircles(centr,rho_o,'Color','b');
%         %viscircles(centers,rho_z,'LineStyle',':','Color','r','LineWidth',.5);
%         viscircles(centers,rho_z,'Color','r','LineWidth',.5);
%         viscircles(centers,rho_f,'LineStyle',':','Color','r','LineWidth',1);

        %call deflect_function to get Fo_n for each obstacle (send in arguments r & r_o)
        %this is a for loop that will generate an array for lambda for each obstacle    
        %add the both attractive and deflective forces 
        [Fo_n , lam] = deflect_functn( r, r_o, r_g);
        %lambda array to be used later
        lam_array(i) = lam;
        %get Fo_star
        Fo_star = Fo_starold + (Fo_n * (1-sig_array(i)));
        Fo_starold = Fo_star;
        hold on
    end

    
    
    %calculates the attractive fields at every point r (by calling the attract fxn)
    [Fg_n] = attract_functn(r, r_g);
    Fg_star = Fg_n * sig_final;
    %disp(Fg_n)
    
    %calculate the linear velocity of the object
    u = k_u * (tanh(norm (r-r_g)));    
    
    %calculate F_star
    F_star = Fg_star + Fo_star;
    Fx_star = F_star(1);
    Fy_star = F_star(2);
    
    %get the si_dot to calculate ang vel, w (the attractive half of F_star)
    lambda = 2;
    r_att = r - r_g;
    get_si_d = get_si_dot (r_att, tetha, lambda, Fx_star, Fy_star, u); 
    si_dot1 = sig_final * get_si_d;
    
    
    %get the si_dot to calculate ang vel, w (the deflective half of F_star)
    si_dot2old = 0;
    %get si_dot2       
    for i = 1 : numberOfObstacles
        lam = lam_array(i);
        r_def = r - obstacles(:,i);
        si_dot_ = get_si_dot (r_def, tetha, lam, Fx_star, Fy_star, u);
        si_dot2 = si_dot2old + (si_dot_ * (1-sig_array(i)));
        si_dot2old = si_dot2;
        
    end    
    
    %calculate si & si_dot
    si = (atan2(Fy_star,Fx_star));
    si_dot = (si_dot1 + si_dot2);
    
    %calc ang vel, w
    w = (-k_w * wrapToPi(tetha-si)) + (si_dot);
    
    %obtain x_dot, y_dot, theta_dot
    ct = cos(tetha);
    st = sin(tetha);
    [A] = [ct,0 ; st,0 ; 0,1] * [u ; w];

    %extract new x & y
    x = A(1) * dt;
    y = A(2) * dt;
    
    %calculate new r
    new_r = r + [x;y]; 

    %update r & tetha
    r =  new_r;  
    new_tetha = (A(3)*dt);
    tetha = wrapToPi(tetha+new_tetha);
    
    %plot every step
    xlim('auto')
    ylim('auto')
    r_centers = r.';
    viscircles(r_centers,rho,'Color','k');
%     plot(r(1),r(2),'k.')
%     hold on
    
%     %displays the circular vehicle at some specified iterations
%     if mod(count,75)== 0 && count < 300
%         r_centers = r.';
%         viscircles(r_centers,rho,'Color','k');
%     elseif mod(count,150)== 0 && count >= 300 && count < 600
%         r_centers = r.';
%         viscircles(r_centers,rho,'Color','k');
%     elseif mod(count,300)== 0 && count >= 600 && count < 1000
%         r_centers = r.';
%         viscircles(r_centers,rho,'Color','k');
%     else
%     end
    hold on
    drawnow()
    count = count + 1;
    
    %update obstacle location constantly
    obstacles = obstacles + (dt*0.0005);
    
    %save frames for video at every 5 iterations
    if mod(count,5)== 0
        xlim('auto')
        ylim('auto')
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
    end
    
end
close(v)
%save image if neccessary
% title('path plan');
% savefig('path plan.fig');


