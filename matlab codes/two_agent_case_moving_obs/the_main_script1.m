clc
clear global

%define time parameters
%tf = 100; % simulation length (seconds)

dt = .5; % time step for integration (seconds)
T = 100000*dt; % measurement time step
%linear and angular velocity gains
k_u = 0.015;
k_w = 2.5;

%create video file
v = VideoWriter('path2.avi');
open(v);

%location of each fixed obstacle
count = 0;

numberOfObstacles = 10; %no. of obstacles

%to create an array of obstacles
%obstacles = [obs1 obs2 obs3 obs4 obs5 obs6 obs7 obs8 obs9 obs10];

%using the mouse to place obstacles



%object radius
rho = 0.03; %predetermined
%obstacle radius
rho_o = 0.03; %predetermined too
%minimum distance between object and obstacle
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

xlim([-.7 0]);
ylim([-.4 .2]);
%define start and end locations and start orientation
%vehicle A
[startxA,startyA] = ginput(1);
r_start = [startxA;startyA];
r1 = r_start; % r is the location of the object at every time, initialized at starting point
plot (r1(1),r1(2), 'bx','MarkerSize', 10,'LineWidth',2);
text((r1(1)+0.01),(r1(2)+0.005),'A','FontWeight','bold')
hold on

%vehicle B
xlim([-.7 0]);
ylim([-.4 .2]);
[startxB,startyB] = ginput(1);
r_start = [startxB;startyB];
r2 = r_start; % r is the location of the object at every time, initialized at starting point
plot (r2(1),r2(2), 'bx','MarkerSize', 10,'LineWidth',2);
text((r2(1)+0.01),(r2(2)+0.005),'B','FontWeight','bold')
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
tetha1 = 0;%(atan2((r_g(2)- r1(2)),(r_g(1)- r1(2))));
tetha2 = 0;%(atan2((r_g(2)- r2(2)),(r_g(1)- r2(2))));
%plot start and end points



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
    viscircles(centers,rho_z,'LineStyle',':');
    viscircles(centers,rho_z,'Color','r');
    viscircles(centers,rho_f,'LineStyle',':');
    
    hold on
end



disp("OBSTACLES COMPLETE");

agentA = 'A';
agentB = 'B';
figure;
for tau = dt : dt : T
    
    for i = 1: numberOfObstacles
        %text(obs_X,obs_Y,num2str(i),'Color','r')
        centers = transpose(obstacles(:,i));
        %axis square
        plot(obs_X,obs_Y,'r.');
        viscircles(centers,rho_o,'Color','b');
%         viscircles(centers,rho_z,'LineStyle',':');
%         viscircles(centers,rho_z,'Color','r');
%         viscircles(centers,rho_f,'LineStyle',':');
        
       
    end
    hold on
    
    %vehicle 1
    [x_dot_A, y_dot_A, tetha_dot_A] = update_position(r1,r_g, rho, rho_e, rho_o, f, tetha1, obstacles,numberOfObstacles,k_u,k_w);
    [r1,tetha1 ] = get_pos(r1,tetha1, x_dot_A, y_dot_A, tetha_dot_A,count, dt,rho,agentA);

    %hold on
    
    %vehicle 2
    %if norm(r2-r1)>(2*(rho_o+rho_e))  && norm(r2-r_g) >= norm(r1-r_g) 
    [x_dot_B, y_dot_B, tetha_dot_B] = update_position(r2,r_g, rho, rho_e, rho_o, f, tetha2, obstacles,numberOfObstacles,k_u,k_w);
    %else
    
    %end
    [r2,tetha2] = get_pos(r2,tetha2, x_dot_B, y_dot_B, tetha_dot_B,count, dt,rho,agentB);
    
    
    obstacles = obstacles + (dt*0.0005);
    count = count + 1;
    %movie creation
    if mod(count,5)== 0
        xlim('auto')
        ylim('auto')
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
    end
    
    %drawnow() 
    if count==300, break, end
    
end
close(v)
