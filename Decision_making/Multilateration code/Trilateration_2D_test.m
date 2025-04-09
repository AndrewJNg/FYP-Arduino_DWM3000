fclose all; close all; clear all; clc;

syms x2 x3 y3 theta 

A1 = [0,0,0];

A2 = [5,0,0];

A3 = [2,2,0];


%%
% Coordinates of the points
x1 = A1(1); y1 = A1(2); 
x2 = A2(1); y2 = A2(2); 
x3 = A3(1); y3 = A3(2);

error_tolerance = 0.1; % 10cm error

error = [rand()*error_tolerance,rand()*error_tolerance,rand()*error_tolerance]

r1 = sqrt(5)    + error(1) ; % A1_to_A2_distance 
r2 = sqrt(10)   + error(2) ;
r3 = 1          + error(3) ;


%% Calculate position

%% 
% Line_bounding(A1,A2,A3,r1,r2,r3)

%%
outputCoord = Trilateration_2D(A1,A2,A3,r1,r2,r3)
x = outputCoord(1); 
y = outputCoord(2);
z = outputCoord(3);




% Extract x and y coordinates
x_true = 2;
y_true = 1;

x_obtained = x;
y_obtained = y;

% Calculate the error distance (Euclidean distance)
error_distance = sqrt((x_obtained - x_true)^2 + (y_obtained - y_true)^2);

% Display the results
disp(['True Value Position: (', num2str(x_true), ', ', num2str(y_true), ')']);
disp(['Obtained Value Position: (', num2str(x_obtained), ', ', num2str(y_obtained), ')']);
disp(['Error Distance: ', num2str(error_distance*100),'cm']);