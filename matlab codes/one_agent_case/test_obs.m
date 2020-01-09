function [ obstacle ] = test_obs (numberOfObstacles, obs_rad)
x = (rand(1, 500000)) * 13;
y = (rand(1, 500000)) * 13;
min_obs_dist = 2 * (obs_rad+0.2);


%initialize obstacle array
obstacle = zeros([2,numberOfObstacles]);

% Initialize first point.
keeperX = x(1);
keeperY = y(1);
% Try dropping down more points.
counter = 1;
for k = 1 : numberOfObstacles
	% Get a trial point.
	thisX = x(k);
	thisY = y(k);
	% See how far is is away from existing keeper points.
	distances = sqrt((thisX-keeperX).^2 + (thisY - keeperY).^2);
	minDistance = min(distances);
    
    if minDistance >= min_obs_dist
		keeperX(counter) = thisX;
		keeperY(counter) = thisY;

		counter = counter + 1;
        
    end
    obs = [thisX;thisY];
    obstacle(:,counter) =  obs;
end
end