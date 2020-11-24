clear;
clc;
filename = "resize_image.png";
A = imread(filename);
B = arrayfun(@(input) ~(input||0), A);
[nrows, ncols] = size(B);
goal = [20, 120];
start = [40, 60];
[x, y] = meshgrid (1:ncols, 1:nrows);
obstacle = B;

%% Compute distance transform

d = bwdist(obstacle);

% Rescale and transform distances

d2 = (d/30) + 1;

d0 = 2;     %3
nu = 400;   %200

repulsive = nu*((1./d2 - 1/d0).^2);
% repulsive = 300*(abs(1./d2 - 1/d0));
repulsive (d2 > d0) = 0;
repulsive(goal(1),goal(2)) = 0;
% a = repulsive(goal(1),goal(2))
%% Display repulsive potential
figure;
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;

title ('Repulsive Potential');

%% Compute attractive force
xi = 1/100;

attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );
% attractive = sqrt( (x - goal(1)).^2 + (y - goal(2)).^2 );
figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;

title ('Attractive Potential');

%% Display 2D configuration space

figure;
imshow(~obstacle);

hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold off;

axis ([0 ncols 0 nrows]);
axis xy;
axis on;

xlabel ('x');
ylabel ('y');

title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;

figure;
m = mesh (f);
m.FaceLighting = 'phong';
axis equal;

title ('Total Potential');

%% Plan route

ncoords = GradientBasedPlanner_SAlm (f, attractive, obstacle, start, goal, 1000);
route = improved_lm(ncoords, obstacle);

%% Plot the energy surface
figure;
m = mesh (f);
axis equal;

%% Plot ball sliding down hill

[sx, sy, sz] = sphere(20);

scale = 3;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
p = mesh(sx, sy, sz);
p.FaceColor = 'red';
p.EdgeColor = 'none';
p.FaceLighting = 'phong';
hold off;

for i = 1:size(route,1)
    P = round(route(i,:));
    z = f(P(2), P(1));
    
    p.XData = sx + P(1);
    p.YData = sy + P(2);
    p.ZData = sz + f(P(2), P(1));
    
    drawnow;
    
    drawnow;
    pause(0.01)
end

%% quiver plot
[gx, gy] = gradient (-f);
skip = 20;

figure;

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 ncols 1 nrows]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
