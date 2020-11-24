%%
% start_coords = [30, 8, 0]; % [y x theta]
% end_coords = [8, 8, 0]; % [y x theta]
% C_size = size(C_space);
% angle_list = -90:5:89;
% route = wavefrontplanner(cost, C_space, start_coords, end_coords);
% 
% for i = 1:length(route)
%     p = route(i,1:2);
%     p_r = reshape(f(p(1), p(2),:),[1, C_size(3)]);
%     [~,index] = min(p_r);
%     angle = angle_list(index);
%     route(i, 3) = angle;
% end
% for i = 1:length(route)
%     box_map(route(i,:), W, b_length, b_width,0.1);
% end
%%
clc;
clear;
b_length = 9;
b_width = 3;
angle_list = -90:5:89;
start_coords = [30, 8, 0]; % [y x theta]
end_coords = [8, 8, 0]; % [y x theta]
goal = [end_coords(1),end_coords(2),angletoindex(end_coords(3))];
[C_space, W] = conv_C_space();
C_size = size(C_space);
%% plot the distance 
[D1, IDX] = bwdist(C_space);
%% attractive field
cost = 1/2 * wavefront_3D(goal, C_space);
%% repulsive field
mu = 100;
D0 = 2;
D2 = D1/10 + 1;
repulsive = mu * ((1./D2 - 1/D0).*(1./D2 > 1/D0)).^2; % wrong repulsive field calculation 
%% add together
f = repulsive + cost;
%% step 1
tic
[S, d1] = S_skeleton(C_space);
toc
%% step 2
au_s = augmentation_s(S, goal, d1);
%% step 3
cost = wavefront_3D_step3(goal, au_s);
%% step 4
cost = wavefront_3D_step4(cost, C_space);
route = wavefrontplanner(cost, C_space, start_coords, end_coords);
%% add the orientation
for i = 1:length(route)
    p = route(i,1:2);
    p_r = reshape(cost(p(1), p(2),:),[1, C_size(3)]);
    [~,index] = min(p_r);
    angle = angle_list(index);
    route(i, 3) = angle;
end
%% path animation
for i = 1:length(route)
    box_map(route(i,:), W, b_length, b_width,0.03);
end






