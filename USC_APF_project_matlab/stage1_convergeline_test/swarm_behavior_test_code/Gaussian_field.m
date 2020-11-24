function [vRef,wRef] = Gaussian_field(pose)
center = [50,50];
alpha = 0; %orientation of the box
D = 10;
line_index = [1 1 0 0];% the line characteristic (converge or not)
line_center = zeros(4,2);
for i = 1:4
    line_center(i,:) = [center(1)+D/2*cos(alpha+(i-1)*pi/2), center(2)+D/2*sin(alpha+(i-1)*pi/2)];
end
%%
% maximum_angle = zeros(100,100);
X_0 = pose(1);
y_0 = pose(2);
epuck_position = [X_0,y_0];
mean_angle = atan2(line_center(:,2)-epuck_position(2),line_center(:,1)-epuck_position(1))';
for i = 1:4
    r(i) = manhal_dis(epuck_position,line_center(i,:));
end
Gaussian_portion = [1/r(1)^2,1/r(2)^2,1/r(3)^2,1/r(4)^2];
sum_portoin = sum(Gaussian_portion);
Gaussian_portion = Gaussian_portion./sum_portoin;
%%
N = zeros(4,8);
for j = 1:4
    angle = -pi:pi/4:pi;
    n = length(angle)-1;
    for i = 1:n
        if angle(i) < -pi+mean_angle(j)
            angle(i) = angle(i) + 2*pi;
        elseif angle(i) > pi+mean_angle(j)
            angle(i) = angle(i) - 2*pi;
        end
        if line_index(j) == 1
            N(j,:) = 2*Gaussian_portion(j)*normpdf(angle(1:8),mean_angle(j),1);
        elseif line_index(j) == 0
            N(j,:) = -1*Gaussian_portion(j)*normpdf(angle(1:8),mean_angle(j),1);
        end
    end
end
S = sum(N,1);
[M,I] = max(S);
maximum_angle = -pi+(I-1)*pi/4;
dangle = maximum_angle - pose(3);
if dangle < -pi
    dangle = dangle+ 2*pi;
elseif dangle > pi
    dangle = dangle -2*pi;
end
vRef = abs(-2*sqrt(sum(r)) + 10);
wRef = dangle;
end