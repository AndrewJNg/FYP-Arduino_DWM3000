function Line_bounding(Coord1,Coord2,Coord3,r1,r2,r3)

% Plot the points and the lines connecting them
figure;
hold on;

% Coordinates of the points
x1 = Coord1(1); y1 = Coord1(2); 
x2 = Coord2(1); y2 = Coord2(2); 
x3 = Coord3(1); y3 = Coord3(2);

% Plot the points
plot(x1, y1, 'ro', 'MarkerFaceColor', 'r'); % Point 1
plot(x2, y2, 'go', 'MarkerFaceColor', 'g'); % Point 2
plot(x3, y3, 'bo', 'MarkerFaceColor', 'b'); % Point 3

% Plot the lines connecting the points
plot([x1 x2], [y1 y2], 'r-'); % Line between point 1 and 2 (r12)
plot([x2 x3], [y2 y3], 'g-'); % Line between point 2 and 3 (r23)
plot([x1 x3], [y1 y3], 'b-'); % Line between point 1 and 3 (r13)

% Annotate distances on the plot
text((x1+x2)/2, (y1+y2)/2, ['r12 = ' num2str(r1)], 'Color', 'r', 'FontSize', 12);
text((x2+x3)/2, (y2+y3)/2, ['r23 = ' num2str(r2)], 'Color', 'g', 'FontSize', 12);
text((x1+x3)/2, (y1+y3)/2, ['r13 = ' num2str(r3)], 'Color', 'b', 'FontSize', 12);

% Set axis equal to preserve scale
axis equal;

% Labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Lines and Distances Between Points');
hold off;
end