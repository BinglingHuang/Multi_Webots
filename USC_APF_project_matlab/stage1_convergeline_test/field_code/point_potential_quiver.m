%% define the environment
clc;
clear;
% position of the goal box
center = [50,50];
alpha = 0; %orientation of the box
D = 10;
line_index = [1 1 0 0];% the line characteristic (converge or not)
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
        strength = [1/r(1)^2,1/r(2)^2,1/r(3)^2,1/r(4)^2];
%         strength = [1/r(1),1/r(2),1/r(3),1/r(4)];
        sum_portoin = sum(strength);
        strength = strength./sum_portoin;
        %%
        vector = zeros(4,2);
        for i = 1:4
            if line_index(i) ==1
                vector(i,:) = 2*strength(i)*[cos(mean_angle(i)),sin(mean_angle(i))];
            else
                vector(i,:) = -1*strength(i)*[cos(mean_angle(i)),sin(mean_angle(i))];
            end
        end
        vector = sum(vector,1);        
        if x>45 && x<55 && y>45 &&y<55
            vector = [0 0];
        end
        
        U(fix(x/5)+1,fix(y/5)+1) = 10*vector(1);
        V(fix(x/5)+1,fix(y/5)+1) = 10*vector(2);
    end
end
quiver(Y,X,U,V);
hold on;
rectangle('Position',[45,45,D,D]);
