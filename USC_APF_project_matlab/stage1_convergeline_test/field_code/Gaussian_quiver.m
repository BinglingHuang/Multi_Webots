%% define the environment
clc;
clear;
% position of the goal box
center = [50,50];
alpha = 0; %orientation of the box
D = 10;
line_index = [1 0 0 1];% the line characteristic (converge or not)
line_center = zeros(4,2);
for i = 1:4
    line_center(i,:) = [center(1)+D/2*cos(alpha+(i-1)*pi/2), center(2)+D/2*sin(alpha+(i-1)*pi/2)];
end
%%
x = 1:5:100;
y = 1:5:100;
[X,Y] = meshgrid(x,y);
U = zeros(20,20);
V = zeros(20,20);
for x = 1:5:100
    for y = 1:5:100
        epuck_position = [x,y];
        mean_angle = atan2(line_center(:,2)-epuck_position(2),line_center(:,1)-epuck_position(1))';
        for i = 1:4
            r(i) = manhal_dis(epuck_position,line_center(i,:));
        end
        Gaussian_portion = [1/r(1)^2,1/r(2)^2,1/r(3)^2,1/r(4)^2];
        sum_portoin = sum(Gaussian_portion);
        Gaussian_portion = Gaussian_portion./sum_portoin;
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
        U(fix(x/5)+1,fix(y/5)+1) = cos(maximum_angle);
        V(fix(x/5)+1,fix(y/5)+1) = sin(maximum_angle);
    end
end
quiver(Y,X,U,V);
hold on;
rectangle('Position',[45,45,D,D]);

