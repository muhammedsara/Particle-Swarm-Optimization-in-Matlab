clc;clear all;close all;
%% initialization
%
vmax = 0.1;
swarm_size = 10;                       % number of the swarm particles
maxIter = 100;                          % maximum number of iterations
stopCondition = 0;                     % variance
inertia = 1;
correction_factor = 2;
E = rand(10,2);                        % set the random [0,1] ranged position of the initial swarm
swarm(1:swarm_size,1,1:2) = E;         % set the position of the particles
swarm(:,2,:) = 0;                      % set initial velocity for particles
swarm(:,4,1) = 9;                      % set the best value so far

%% define the objective funcion
objfcn = @(x)(1+ 2 - 0.5 * (x(:,1)+x(:,2))).^(2 - 0.5 * (x(:,1)+x(:,2)));
tic;
%% The main loop of PSO
for iter = 1:maxIter
    swarm(:, 1, 1) = swarm(:, 1, 1) + swarm(:, 2, 1)/1.3;       %update x1 position with the velocity
    swarm(:, 1, 2) = swarm(:, 1, 2) + swarm(:, 2, 2)/1.3;       %update x2 position with the velocity
    
    %Range of particles [0,1]
     indis = find(swarm(:, 1, 1) > 1);
     swarm(indis, 1, 1) = 1;
     indis = [];
     indis = find(swarm(:, 1, 2) > 1);
     swarm(indis, 1, 2) = 1;
     indis = [];
     indis = find(swarm(:, 1, 1) < 0);
     swarm(indis, 1, 1) = 0;
     indis = [];
     indis = find(swarm(:, 1, 2) < 0);
     swarm(indis, 1, 2) = 0;
     indis = [];
     
    x1 = swarm(:, 1, 1);                                         % get the updated position
    x2 = swarm(:, 1, 2);                                         % updated position
    fval = objfcn([x1 x2]);                                      % evaluate the function using the position of the particle
    
    % compare the function values to find the best ones
    for ii = 1:swarm_size
        if fval(ii,1) < swarm(ii,4,1)
           
            swarm(ii, 3, 1) = swarm(ii, 1, 1);                  % update pbest x1 position,
            swarm(ii, 3, 2) = swarm(ii, 1, 2);                  % update pbest x2 postions
            swarm(ii, 4, 1) = fval(ii,1);                       % update the best value so far
            
        end
    end
    
    [~, gbest] = min(swarm(:, 4, 1));                           % find the best function value in total
    
    % update the velocity of the particles
    swarm(:, 2, 1) = inertia*(rand(swarm_size,1).*swarm(:, 2, 1)) + correction_factor*(rand(swarm_size,1).*(swarm(:, 3, 1) ...
        - swarm(:, 1, 1))) + correction_factor*(rand(swarm_size,1).*(swarm(gbest, 3, 1) - swarm(:, 1, 1)));   %x1 velocity component
    swarm(:, 2, 2) = inertia*(rand(swarm_size,1).*swarm(:, 2, 2)) + correction_factor*(rand(swarm_size,1).*(swarm(:, 3, 2) ...
        - swarm(:, 1, 2))) + correction_factor*(rand(swarm_size,1).*(swarm(gbest, 3, 2) - swarm(:, 1, 2)));   %x2 velocity component
    
    % control vmax
     indis = find(swarm(:, 2, 1)>vmax);
     swarm(indis, 2, 1) = vmax;
     indis = [];
     indis = find(swarm(:, 2, 2)>vmax);
     swarm(indis, 2, 2) = vmax;
     indis = [];
     
    % plot the gbest points
    figure(1);
    clf;plot3(swarm(gbest, 1, 2), swarm(gbest, 1, 1), swarm(gbest,4,1), 'rx'); grid;             % drawing swarm movements
    axis([0 1 -0 1 1 4]);
    xlabel('x1');
    ylabel('x2');
    zlabel('gbest (Best Value So Far)');
    pause(.001);                                                 
    disp(['iteration: ' num2str(iter)]);
    %disp(['Best Value: ' num2str(swarm(gbest,4,1))]);                   % un-comment this line to see the best value so far for every iteration
    
    %stop condition
    if(var(swarm(:,1,:)) == stopCondition)
    disp('Stop Condition: Parçacýklarýn tüm deðerleri ayný noktada toplandý.');
    disp(['Best Value: ' num2str(swarm(gbest,4,1))]);
    break;
    end
    
end
toc