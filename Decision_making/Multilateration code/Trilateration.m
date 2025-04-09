fclose all; close all; clear all; clc;

syms x2 x3 y3 theta 

A1 = [0,0,0];

A2 = [x2,0,0];

A3 = [x3,y3,0];

% r12 = 5; % A1_to_A2_distance 
% r23 = 5;
% r13 = 5;

% r12 = 4; % A1_to_A2_distance 
% r23 = sqrt(8);
% r13 = sqrt(8);

r12 = 5; % A1_to_A2_distance 
r23 = sqrt(13);
r13 = sqrt(8);

%% 

A2(1) = r12;

m = (r12^2 - r23^2 - r13^2) / (2*r23 * r13);
theta = acosd(m);

C = asind(r23 * sind(theta) /r12)
B = asind(r13 * sind(theta) /r12)

A3(1) = r13*sind(C);
A3(2) = r23*sind(B);


%%
% Coordinates of the points
x1 = A1(1); y1 = A1(2); 
x2 = A2(1); y2 = A2(2); 
x3 = A3(1); y3 = A3(2);

% Radii of the circles
r1 = r12; 
r2 = r13; 
r3 = r23;



% % Coordinates of the points
% x1 = 2; y1 = 2; 
% x2 = 6; y2 = 2; 
% x3 = 4; y3 = 4;
% 
% % Radii of the circles
% r1 = r12; 
% r2 = r13; 
% r3 = r23;

%% 
% Line_bounding(A1,A2,A3,r12,r23,r13)

%%
Circle_bounding(A1,A2,A3,r12,r23,r13)
