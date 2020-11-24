clc;
clear;
p = zeros(100,100);
p(:,1) = 1;
p(1,:) = 1;
p(:,100) = 1;
p(100,:) = 1;
p(45:55,45:55) = 1;
map = occupancyMap(p);
show(map)
