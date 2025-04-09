fclose all; close all; clear all; clc;% Load data with original column names
% Main script
global global_noise_factor;
global global_noise_range;

global_noise_range = 0.01;
global_noise_factor = 0.0;


% Initial positions of robots
robot_start_positions = [-3 -2; 2 -2; -2 2; 2 2]
% robot_start_positions = [0 0; 5 0; 3 3; 0 5]
%       ; 2 2; 5 1; 0 2; 4 2];  % Each row is [x, y] for one robot 

positions = robot_start_positions;
figure;
hold on;
for i = 1:size(positions, 1)
    plot(positions(i,1), positions(i,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(positions(i,1)+0.1, positions(i,2), sprintf('R%d', i), 'FontSize', 12);
end
grid on;
xlabel('X');
ylabel('Y');
hold off;
title('Initial Robot Positions');
axis equal;

robot_movement= [3 1 1;
                 1 3 1;
                 2 1 2;
                 3 2 1];

%% Start conditions
% Plot initial positions
estimated_positions = robot_start_positions;  % Robot_positions would be a delinked version that shows what the robots think their location is
actual_positions = robot_start_positions;


%% Movement
% Movement of robots one step at a time

% estimated_positions = calculated_move_displacement(estimated_positions, 3, 1,1, 0.5);

robot_id = 3;
dx = 5; 
dy = 5;

[actual_positions,estimated_positions] = move_to_target(actual_positions,estimated_positions, 1, -1, -1, 0.1);
[actual_positions,estimated_positions] = move_to_target(actual_positions,estimated_positions, 2, 1, -1, 0.1);
[actual_positions,estimated_positions] = move_to_target(actual_positions,estimated_positions, 3, -1, 1, 0.1);
[actual_positions,estimated_positions] = move_to_target(actual_positions,estimated_positions, 4, 1, 1, 0.1);

% for i = 1:10
%     estimated_positions = move_to_target(actual_positions,estimated_positions, 1, -1, -1, 0.1);
%     estimated_positions = move_to_target(actual_positions,estimated_positions, 2, 1, -1, 0.1);
%     estimated_positions = move_to_target(actual_positions,estimated_positions, 3, -1, 1, 0.1);
%     estimated_positions = move_to_target(actual_positions,estimated_positions, 4, 1, 1, 0.1);
% end


%     % Obtain position from sensor after other robots moved 
% %     distances = computeDistances(actual_positions);
% %     robot_received_data = getNeighborData(estimated_positions, distances, robot_id); % Get neighbour data
% %     estimated_positions(robot_id,:) = Trilateration_2D(robot_received_data);    % calculate own position
% 
%     plotRobotsAfter(estimated_positions);
% 
%     actual_positions = moveRobot(actual_positions, robot_id, dx, dy);
% 
%     % Estimate it's movement again
%     distances = computeDistances(actual_positions);
%     robot_received_data = getNeighborData(estimated_positions, distances, robot_id); % Get neighbour data
%     estimated_positions(robot_id,:) = Trilateration_2D(robot_received_data);    % calculate own position
% 
% 
% 
% % Obtain position from sensor
%     distances = computeDistances(actual_positions);
%     robot_received_data = getNeighborData(estimated_positions, distances, robot_id); % Get neighbour data
%     estimated_positions(robot_id,:) = Trilateration_2D(robot_received_data);    % calculate own position
% 
    plotRobotsAfter(estimated_positions);





%     robot_positions = moveRobot(robot_positions, robot_id, dx, dy);
%     robot_received_data = getNeighborData(robot_positions, distances, robot_id); % Get neighbour data
%     [x, y] = Trilateration_2D(robot_received_data);    % calculate own position
%     robot_positions(robot_id,:) = [x,y];                % Update own position 



% robot_positions = calculated_move_displacement(robot_positions, 1, 3,1, 1);
% robot_positions = calculated_move_displacement(robot_positions, 2, 1,2, 1);
% robot_positions = calculated_move_displacement(robot_positions, 3, 2,1, 1);



%% Final position plotting
% positions = actual_positions;
positions = estimated_positions;
for i = 1:size(positions, 1)
    plot(positions(i,1), positions(i,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end



%%
function [actual_positions,robot_positions] = move_to_target(actual_positions,robot_positions, robot_id, target_x, target_y, step_size)
    % Step 1: Estimate current position using trilateration
    distances = computeDistances(actual_positions);
    neighbor_data = getNeighborData(actual_positions, distances, robot_id); 
    [x_est, y_est] = Trilateration_2D(neighbor_data);
    robot_positions(robot_id, :) = [x_est, y_est];  % Update estimated position
    
    % Step 2: Compute total displacement
    total_dx = target_x - x_est;
    total_dy = target_y - y_est;
    total_distance = norm([total_dx, total_dy]);
    
    % Step 3: Calculate number of steps
    steps = max(1, round(total_distance / step_size));
    
    % Step 4: Compute displacement per step
    dx_step = total_dx / steps;
    dy_step = total_dy / steps;
    
    % Step 5: Interpolated movement
    for i = 1:steps
        actual_positions = calculated_move(actual_positions, robot_id, dx_step, dy_step);
    end

    % Step 6: Final position update using trilateration
    distances = computeDistances(robot_positions);
    neighbor_data = getNeighborData(robot_positions, distances, robot_id);
    [x_est, y_est] = Trilateration_2D(neighbor_data);
    robot_positions(robot_id, :) = [x_est, y_est];
    
    % Optional: plot after movement
    plotRobotsAfter(actual_positions);
end

function robot_positions = calculated_move(robot_positions, robot_id, dx,dy)
    robot_positions = moveRobot(robot_positions, robot_id, dx, dy);
    distances = computeDistances(robot_positions);
    robot_received_data = getNeighborData(robot_positions, distances, robot_id); % Get neighbour data
    [x, y] = Trilateration_2D(robot_received_data);    % calculate own position
    robot_positions(robot_id,:) = [x,y];                % Update own position 
    plotRobotsAfter(robot_positions);
end




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
    global global_noise_range;  % Declare the global variable
% Distance + 10cm noise 
    n = size(positions, 1);
    distances = zeros(n);
%     noise_range = 0;  % 10 cm noise (in meters)
%     noise_range = 0.1;  % 10 cm noise (in meters)
    noise_range = global_noise_range;

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
    global global_noise_factor;  % Declare the global variable
    % positions: current robot positions [n x 2]
    % robot_id: index of the robot being moved
    % dx, dy: movement in x and y directions
    % noise_factor: a small value representing how much noise is proportional to movement
%     noise_factor=0.01;
%     noise_factor=0.01;
    noise_factor = global_noise_factor;

    % Calculate the updated position
    updated_positions = positions;
    updated_positions(robot_id, 1) = updated_positions(robot_id, 1) + dx;
    updated_positions(robot_id, 2) = updated_positions(robot_id, 2) + dy;

    % Calculate the noise based on movement distance (proportional to dx, dy)
    noise_x = noise_factor * dx * (2*rand - 1);  % Random noise in x (range: -dx to +dx)
    noise_y = noise_factor * dy * (2*rand - 1);  % Random noise in y (range: -dy to +dy)

    % Apply the noise to the updated positions
    updated_positions(robot_id, 1) = updated_positions(robot_id, 1) + noise_x;
    updated_positions(robot_id, 2) = updated_positions(robot_id, 2) + noise_y;
end