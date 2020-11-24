%% before running this code, run test_1 first.
clc;
clear;
b_length = 9;
b_width = 3;
[C_space, W] = conv_C_space();
initial = [33, 9, 0]; % initial position in [Y, X, THETA] format
Qg = [8, 8, 0]; % goal position in [Y, X, THETA] format
parent = [1]; % initializing array to hold the index of the parent node 
size_map = [40, 40, 180];
steps = 100;
t_step = 1 / steps;
if C_space(initial(1),initial(2),angletoindex(initial(3))), error("initial position lies on an obstacle"); end
if C_space(Qg(1),Qg(2),angletoindex(Qg(3))), error("goal position lies on an obstacle"); end
RRTree = double([initial -1]);
maxattempts = 1e4;
%% starting RRT
for i = 1:maxattempts
    coin = rand(1) * 100;
    if (coin <= 80)
        Qrand = round(rand(1,3).*size_map);
    else
        Qrand = Qg;
    end
    % selects the node in the RRT tree that is closet to qrand
    [A, I] = min(metric(RRTree(:,1:3), Qrand),[],1);
    Qnear = RRTree(I(1), 1:3);
    %Qnew<--Stopping Configuration (Qnear, Qrand)
    t=0;
    tmax=0;
    %linear interpolation between Qnear and Qrand
    %stepping from Qnear towards Qrand 
    while(t<1)
        xtemp=(1-t)*Qnear(1)+(t)*Qrand(1);
        ytemp=(1-t)*Qnear(2)+(t)*Qrand(2);
        delta_theta = abs(Qnear(3)-Qrand(3));
        if delta_theta <= 90
            theta_temp = (1-t)*Qnear(3)+(t)*Qrand(3);
        else
            Qnear(3) = Qnear(3)*(Qnear(3)<=90) + (Qnear(3)-180)*(Qnear(3)>90);
            Qrand(3) = Qrand(3)*(Qrand(3)<=90) + (Qrand(3)-180)*(Qrand(3)>90);
            theta_temp = (1-t)*Qnear(3)+(t)*Qrand(3);
            theta_temp = ang_range(theta_temp);
        end
        Qtemp=round([xtemp ytemp theta_temp]);
        if ~C_space(Qtemp(1),Qtemp(2),angletoindex(Qtemp(3)))          % if no collision
            tmax=t;
            Qnew=Qtemp;
        else                                % if collision
            break
        end
        t=t+t_step;
    end  
    % compare if the new node is in the RRTree
    comp = sum(abs(RRTree(:,1:3) - Qnew),2);
    if find(comp == 0)
        continue
    end
    
    if(tmax~=0)                             
        RRTree=[RRTree; Qnew I(1)];   
        if(Qnew == Qg)      % check if the added node is the goal node
            found = 1
            break;        % terminate if goal node is added
        end
    end
end
%% 
path = [Qg];
index = RRTree(end,4);
while (index ~= -1)
    path = [path; RRTree(index,1:3)];
    index = RRTree(index,4);
end
%% draw the imrpoved path
path = path(end:-1:1,:);
improved_path = improved_RRT(path, C_space);
for i = 1:length(improved_path)
    box_map(improved_path(i,:), W, b_length, b_width,0.01);
end

