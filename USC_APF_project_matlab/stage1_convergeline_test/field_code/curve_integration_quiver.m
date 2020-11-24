clc;
clear;
% position of the goal box
center = [50,50];
alpha = 0; %orientation of the box
D = 10;
line_index = [0 0 1 0];% the line characteristic (converge or not)
line_center = zeros(4,2);
line_point = zeros(4,2);
line_k = [0 0 0 0];
a = 0;
for i = 1:4
    line_center(i,:) = [center(1)+D/2*cos(alpha+(i-1)*pi/2), center(2)+D/2*sin(alpha+(i-1)*pi/2)];
end
%%
%% line1
for i =1:4
    line_point(i,:) = [center(1)+D*sqrt(2)/2*cos(pi/4+alpha+(i-2)*pi/2), center(2)+D*sqrt(2)/2*sin(pi/4+alpha+(i-2)*pi/2)];
end
for i =1:4
    if i == 4
        a = 1;
    else
        a = i+1;
    end
    line_k(i) = (line_point(a,2)-line_point(i,2))/(line_point(a,1)-line_point(i,1)+eps)+eps;
end
%%
x_0 = 1:5:100;
y_0 = 1:5:100;
[X,Y] = meshgrid(x_0,y_0);
U = zeros(20,20);
V = zeros(20,20);
sum_x = 0;
sum_y = 0;

for x_0= 1:5:100
    for y_0 = 1:5:100
        epuck_position = [x_0,y_0];
        total_x=0;
        total_y=0;
        for i = 1:4
            if(i==4)
                a = 1;
            else
                a = i+1;
            end
            if(line_index(i)==1)
                if abs(line_k(i)) < 1e1
                    r_x = @(x) ((x-x_0).^2 + (line_k(i).*(x-line_point(i,1))+line_point(i,2)-y_0).^2);
                    fun1 = @(x) ((x-x_0)./r_x(x).* sqrt(1+line_k(i)^2./r_x(x)));
                    fun2 = @(x) ((line_k(i).*(x-line_point(i,1))+line_point(i,2)-y_0)./r_x(x).* sqrt(1+line_k(i)^2./r_x(x)));
                    sum_x = 4*sign(line_point(a,1)-line_point(i,1))*integral(fun1,line_point(i,1),line_point(a,1));
                    sum_y = 4*sign(line_point(a,1)-line_point(i,1))*integral(fun2,line_point(i,1),line_point(a,1));
                end
                if abs(line_k(i)) > 1e1
                    r_y = @(y) ((1/line_k(i)*(y-line_point(i,2))+line_point(i,1)-x_0).^2 + (y-y_0).^2);
                    fun1 = @(y) ((1/line_k(i)*(y-line_point(i,2))+line_point(i,1)-x_0)./r_y(y).* sqrt(1+(1/line_k(i))^2./r_y(y)));
                    fun2 = @(y) ((y-y_0)./r_y(y).* sqrt(1+(1/line_k(i))^2./r_y(y)));
                    sum_x = 4*sign(line_point(a,2)-line_point(i,2))*integral(fun1,line_point(i,2),line_point(a,2));
                    sum_y = 4*sign(line_point(a,2)-line_point(i,2))*integral(fun2,line_point(i,2),line_point(a,2));
                end
                total_x = total_x + sum_x;
                total_y = total_y + sum_y;
            elseif(line_index(i)==0)
                if abs(line_k(i)) < 1e1
                    r_x = @(x) ((x-x_0).^2 + (line_k(i).*(x-line_point(i,1))+line_point(i,2)-y_0).^2);
                    fun1 = @(x) ((x-x_0)./r_x(x).* sqrt(1+line_k(i)^2./r_x(x)));
                    fun2 = @(x) ((line_k(i).*(x-line_point(i,1))+line_point(i,2)-y_0)./r_x(x).* sqrt(1+line_k(i)^2./r_x(x)));
                    sum_x = -1*sign(line_point(a,1)-line_point(i,1))*integral(fun1,line_point(i,1),line_point(a,1));
                    sum_y = -1*sign(line_point(a,1)-line_point(i,1))*integral(fun2,line_point(i,1),line_point(a,1));
                end
                if abs(line_k(i)) > 1e1
                    r_y = @(y) ((1/line_k(i)*(y-line_point(i,2))+line_point(i,1)-x_0).^2 + (y-y_0).^2);
                    fun1 = @(y) ((1/line_k(i)*(y-line_point(i,2))+line_point(i,1)-x_0)./r_y(y).* sqrt(1+(1/line_k(i))^2./r_y(y)));
                    fun2 = @(y) ((y-y_0)./r_y(y).* sqrt(1+(1/line_k(i))^2./r_y(y)));
                    sum_x = -1*sign(line_point(a,2)-line_point(i,2))*integral(fun1,line_point(i,2),line_point(a,2));
                    sum_y = -1*sign(line_point(a,2)-line_point(i,2))*integral(fun2,line_point(i,2),line_point(a,2));
                end
                total_x = total_x + sum_x;
                total_y = total_y + sum_y;
            end
        end 
        if x_0>45 && x_0<55 && y_0>45 &&y_0<55
            total_x = 0;
            total_y = 0;
        end
        U((x_0+4)/5,(y_0+4)/5) = total_x;
        V((x_0+4)/5,(y_0+4)/5) = total_y;  
    end
        
end
figure(1);
quiver(Y,X,U,V);
hold on;
rectangle('Position',[45,45,D,D]);
        
        
