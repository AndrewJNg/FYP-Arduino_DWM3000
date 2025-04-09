function [outputCoord] = Trilateration_2D(Coord1,Coord2,Coord3,r1,r2,r3)

% Angles for parametric equations
theta = linspace(0, 2*pi, 100); % 100 points to make the circle smooth


% Coordinates of the points
x1 = Coord1(1); y1 = Coord1(2); 
x2 = Coord2(1); y2 = Coord2(2); 
x3 = Coord3(1); y3 = Coord3(2);



% Output coordinate
x = (r1^2 -r2^2+x2^2)/(2*x2);
y = (r1^2 -r3^2+x3^2+y3^2-(2*x3*x))/(2*y3);
z = real(sqrt(r1^2-x^2-y^2));


% Circle 1
x_circle1 = x1 + r1 * cos(theta);
y_circle1 = y1 + r1 * sin(theta);

% Circle 2
x_circle2 = x2 + r2 * cos(theta);
y_circle2 = y2 + r2 * sin(theta);

% Circle 3
x_circle3 = x3 + r3 * cos(theta);
y_circle3 = y3 + r3 * sin(theta);

% Plot the circles
figure(1);
hold on;
plot(x_circle1, y_circle1, 'r'); % Circle 1 in red
plot(x_circle2, y_circle2, 'g'); % Circle 2 in green
plot(x_circle3, y_circle3, 'b'); % Circle 3 in blue

% Plot the center points
plot(x1, y1, 'ro', 'MarkerFaceColor', 'r'); % Center of circle 1
plot(x2, y2, 'go', 'MarkerFaceColor', 'g'); % Center of circle 2
plot(x3, y3, 'bo', 'MarkerFaceColor', 'b'); % Center of circle 3

% Set axis equal for a proper circle shape
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
title('Circles with Given Radii around Points');

plot(x, y, 'ko', 'MarkerFaceColor', 'k'); % Point 1
hold off;


outputCoord = [x,y,z];
end