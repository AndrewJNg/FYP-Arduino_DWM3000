fclose all; close all; clear all; clc;% Load data with original column names
% Main script

% Initial positions of robots
robot_start_positions = [0 0; 5 0; 3 3; 0 5];  % Each row is [x, y] for one robot 

%% Start conditions
% Plot initial positions
robot_positions = robot_start_positions;  % Robot_positions would be a delinked version that shows what the robots think their location is

figure;
plotRobots(robot_positions);
title('Initial Robot Positions');
axis equal;


%% Movement
% Compute pairwise distances
distances = computeDistances(robot_positions);
disp('Initial Distances Between Robots:');
disp(distances);


robot_id = 3
robot_received_data = getNeighborData(robot_positions, distances, robot_id) % Get neighbour data
[x, y] = Trilateration_2D(robot_received_data);    % calculate own position
robot_positions(robot_id,:) = [x,y]                % Update own position 



% Example move: move robot 1 by dx = 1, dy = -0.5
robot_id = 3;
dx = 1;
dy = 0.5;
robot_positions = moveRobot(robot_positions, robot_id, dx, dy)
distances = computeDistances(robot_positions);
robot_received_data = getNeighborData(robot_positions, distances, robot_id) % Get neighbour data
[x, y] = Trilateration_2D(robot_received_data);    % calculate own position
robot_positions(robot_id,:) = [x,y]                % Update own position 

robot_id = 2;
dx = -3;
dy = 0.5;
robot_positions = moveRobot(robot_positions, robot_id, dx, dy)
distances = computeDistances(robot_positions);
robot_received_data = getNeighborData(robot_positions, distances, robot_id) % Get neighbour data
[x, y] = Trilateration_2D(robot_received_data);    % calculate own position
robot_positions(robot_id,:) = [x,y]                % Update own position 
 
plotRobotsAfter(robot_positions);


%% Final
% Compute new distances
distances = computeDistances(robot_positions);
disp('Distances After Movement:');
disp(distances);



%% Function for trilateration
function data = getNeighborData(robot_positions, distances, robot_id)
    % robot_positions: n x 2 matrix [x, y]
    % distances:       n x n matrix (distance from i to j)
    % robot_id:        index of the current robot (1 to n)

    n = size(robot_positions, 1);
    
    if robot_id > n || robot_id < 1
        error('robot_id is out of bounds.');
    end
    
    % Exclude the current robot
    idx = setdiff(1:n, robot_id);
    
    % Get neighbor positions and distances
    neighbor_positions = robot_positions(idx, :);
    neighbor_distances = distances(robot_id, idx)';
    
    % Combine into [x, y, d] format
    data = [neighbor_positions, neighbor_distances];
end



%% Function for trilateration
function [x, y] = Trilateration_2D(data)
    n = size(data, 1);
    
    if n < 2
        error('At least two reference points are required.');
    elseif n == 2
        % Two-circle intersection (choose one of the two possible points)
        [x1, y1, d1] = deal(data(1,1), data(1,2), data(1,3));
        [x2, y2, d2] = deal(data(2,1), data(2,2), data(2,3));
        
        D = sqrt((x2 - x1)^2 + (y2 - y1)^2);

        if D > d1 + d2 || D < abs(d1 - d2) || D == 0
            error('No solution: the circles do not intersect or are coincident.');
        end

        a = (d1^2 - d2^2 + D^2) / (2 * D);
        px = x1 + a * (x2 - x1) / D;
        py = y1 + a * (y2 - y1) / D;
        h = sqrt(d1^2 - a^2);
        rx = -(y2 - y1) * (h / D);
        ry =  (x2 - x1) * (h / D);

        x = px + rx;  % First possible solution
        y = py + ry;
        
    else
        % Least-squares trilateration for 3+ references
        A = [];
        b = [];
        x1 = data(1,1);
        y1 = data(1,2);
        d1 = data(1,3);

        for i = 2:n
            xi = data(i,1);
            yi = data(i,2);
            di = data(i,3);
            
            A_i = [2*(xi - x1), 2*(yi - y1)];
            b_i = d1^2 - di^2 + xi^2 - x1^2 + yi^2 - y1^2;
            
            A = [A; A_i];
            b = [b; b_i];
        end

        % Least-squares solution
        pos = (A\b);
        x = pos(1);
        y = pos(2);
    end
end



%% Function to plot robots
function plotRobots(positions)
    hold on;
    for i = 1:size(positions, 1)
        plot(positions(i,1), positions(i,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        text(positions(i,1)+0.1, positions(i,2), sprintf('R%d', i), 'FontSize', 12);
    end
    grid on;
    xlim([-1 5]);
    ylim([-1 5]);
    xlabel('X');
    ylabel('Y');
    hold off;
end
%% Function to plot robots
function plotRobotsAfter(positions)
    hold on;
    for i = 1:size(positions, 1)
        plot(positions(i,1), positions(i,2), 'bo', 'MarkerSize', 4, 'LineWidth', 2);
%         text(positions(i,1)+0.1, positions(i,2), sprintf('R%d', i), 'FontSize', 12);
    end
%     grid on;
%     xlim([-1 5]);
%     ylim([-1 5]);
%     xlabel('X');
%     ylabel('Y');
%     hold off;
end

%% Function to compute pairwise distances between robots
function distances = computeDistances(positions)
%     n = size(positions, 1);
%     distances = zeros(n);
%     for i = 1:n
%         for j = 1:n
%             if i ~= j
%                 dx = positions(i,1) - positions(j,1);
%                 dy = positions(i,2) - positions(j,2);
%                 distances(i,j) = sqrt(dx^2 + dy^2);
%             end
%         end
%     end

% Distance + 10cm noise 
    n = size(positions, 1);
    distances = zeros(n);
    noise_range = 0;  % 10 cm noise (in meters)
%     noise_range = 0.1;  % 10 cm noise (in meters)

    for i = 1:n
        for j = 1:n
            if i ~= j
                dx = positions(i,1) - positions(j,1);
                dy = positions(i,2) - positions(j,2);
                true_dist = sqrt(dx^2 + dy^2);

                % Add uniform noise between -0.1 and +0.1
                noise = (rand() - 0.5) * 2 * noise_range;
                distances(i,j) = true_dist + noise;
            end
        end
    end

end

%% Function to move a robot
function updated_positions = moveRobot(positions, robot_id, dx, dy)
    updated_positions = positions;
    updated_positions(robot_id, 1) = updated_positions(robot_id, 1) + dx;
    updated_positions(robot_id, 2) = updated_positions(robot_id, 2) + dy;
end