function [C_space, W] = conv_C_space()
    %% read the map
    clear;
    clc;
    filename = "Black_white_image.png";
    pic = imread(filename);
    pic = imresize(pic, 0.25);
    pic = imresize(pic, 0.25);
    [r_x, r_y] = size(pic);
    %% resize the map
    B = arrayfun(@(input) ~(input||0), pic);
    [nrows, ncols] = size(B);
    [x, y] = meshgrid (1:ncols, 1:nrows);
    W = B;    %% obstalce is "0" free space is "0"

    W(1:end, 1) = 1;
    W(1, 1:end) = 1;
    W(end, 1:end) = 1;
    W(1:end, end) = 1;

    %% FT OF W
    W = flipud(W);
    W_FT = fft2(W);
    N = size(W_FT);
    %% define box size
    b_length = 9;
    b_width = 3;
    Num_degree = 36;
    fix_point = [1 1];
    A_one_one = zeros(N);
    % A_one_one(1:3, 1:7) = 1;
    origin_box = [];
    for i = 0:2
        for j = -3:5
            index_init = [i,j];
            origin_box = [origin_box; index_init];
            index_init = limit_amplitude(index_init,N);
            A_one_one(index_init(1), index_init(2)) = 1;
        end
    end
    %% for loop
    num = 1;
    C_space = zeros(N(1), N(2), Num_degree);
    for theta = -90:(180/Num_degree):89
        A_one_one_theta = zeros(N);
        A_theta = zeros(N);
        [row,col] = find(A_one_one);
        Index = [row,col];
        rad = theta * pi/180;
        R = [cos(rad) -1 * sin(rad); sin(rad) cos(rad)];
        [Num,m] = size(Index);
        for i = 1:Num
    %         delta = Index(i,:)' - fix_point';
            delta = origin_box(i,:)' - fix_point';
            P_new = R*delta + fix_point';
            P_new = round(P_new);
            P_new = limit_amplitude(P_new, N);
            A_one_one_theta(P_new(1),P_new(2)) = 1;
            A_theta(1 - P_new(1) + N(1), 1 - P_new(2) + N(2)) = 1;
        end
        A_theta_FT = fft2(A_theta);
        X = W_FT.*A_theta_FT;
        Y = ifft2(X);
        Y = arrayfun(@(input) input>=0.2, Y);
        Y = [Y(end,:);Y(1:end-1,:)];
        Y = [Y(:,end) Y(:,1:end-1)];
        C_space(:,:,num) = Y;
        num = num + 1;
    end
end