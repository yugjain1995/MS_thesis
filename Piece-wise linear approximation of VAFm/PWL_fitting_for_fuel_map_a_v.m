% This script performs piece-wise linear approximation of fuel rate map
% Algorithm used - Convex piecewise-linear fitting
% Alessandro Magnani , Stephen P. Boyd

% clear;clc;close all;
import Partition_class
gear = 6;
% VAFm = VAFm*3600; % g/sec -> g/hr

load('GM_9T50_M3DandH_LTG_and_Final_Drive_powertrain_data.mat');
vel_lim_array = transmission_data.gear_velocity_range;
vel_lim = vel_lim_array(gear,:);

K_max = 4; % Maximum number of piece wise partitions of the fuel rate map
N_trials = 10; % Maximum number of trials for fitting optimization
l_max = 50; % Maximum number of iteratios in each trial


% Data set (Data set is 40 x 40 = 1600)
% row_of_ind = (1,1);(1,2);(1,3);.....(1,40);(2,1);....(2,40);(40,1);....(40,40)
% 1st index is for torque and 2nd index for speed


%% Index matrix for all points in map

row_of_ind = [];

for i = 1:40
    for j = 1:40
        if(~isnan(VAFm(i,j)) && VehSpd(i) >= vel_lim(1) && VehSpd(i) < vel_lim(2)) % Only include the data point is VAFm is not NaN and velocity limits of the gear is obeyed
            row_of_ind = [row_of_ind; [i, j]]; % Index i corresponds to velocity and index j corresponds to acceleration
        end
    end
end




results(1:K_max) = results_collector;
%% Running the algorithm
for K = 4:K_max
    rms_N = 10^5; % Initialize it with big value so that it will select the 1st value when starting the trials
    for N = 1:N_trials
        K_act = K;
        %% Initial Partitioning
        %% Choose random K_act row numbers of row_of_ind
        r = randperm(size(row_of_ind,1),K_act);
        %% Generating array of K_act partition class corresponding to K_act partition points
        P_class(1:K_act) = Partition_class;
        
        %% Initializing partitions
        for i = 1:K_act
            P_class(i).P_ind = row_of_ind(r(i),:); % Assigning Partition point to each Partition class object
            P_class(i).P_val = [VehSpd(row_of_ind(r(i),1)), VehAccReq(row_of_ind(r(i),2)), VAFm(row_of_ind(r(i),1),row_of_ind(r(i),2))]; % Assigning Partition point value to each Partition class object
        end
        
        %% Distributing all points to nearest partition point
        for i = 1:size(row_of_ind,1)
            point = [VehSpd(row_of_ind(i,1)), VehAccReq(row_of_ind(i,2)), VAFm(row_of_ind(i,1),row_of_ind(i,2))];
            d = zeros(K_act,1);
            for j = 1:K_act
                d(j) = P_class(j).eucledian_dist(point);
            end
            [~,min_d_ind] = min(d);
            P_class(min_d_ind).P_set.val = [P_class(min_d_ind).P_set.val;point];
            P_class(min_d_ind).P_set.ind = [P_class(min_d_ind).P_set.ind;row_of_ind(i,:)];
        end
        
        %% Removing partition if no point in it and putting the corresponding partition point to other nearest partition set
        for  i = 1:K_act
            if(isempty(P_class(i).P_set))
                P_val = P_class(i).P_val;
                P_ind = P_class(i).P_ind;
                P_class(i) = [];
                K_act = K_act-1;
                d = [];
                for j = 1:K_act
                    if(~isempty(P_class(j).P_set))
                        d = [d;P_class(j).eucledian_dist(P_val)];
                    else
                        d = [d;10^5]; %If set we are measuring the dist from is empty than put a huge dist value so that it will be rejected as it is not min
                    end
                end
                [~,min_d_ind] = min(d);
                P_class(min_d_ind).P_set.val = [P_class(min_d_ind).P_set.val;P_val];
                P_class(min_d_ind).P_set.ind = [P_class(min_d_ind).P_set.ind;P_ind];
            end
        end
        
        
        
        
        %% Re-partitioning and fitting iterations
        for l = 1:l_max
            %% Calculation of linear fit (a1.VehSpd + a2.VehAccReq + a3) for each partition
            
            for i = 1:K_act
                P_class(i) = P_class(i).partition_fit;
                P_class(i) = P_class(i).replace_set; % Placing the current set in old set and emptying the current set
            end
            %         [temp,rms_error(1)] = lsqlin([P1_set(:,1:2),ones(size(P1_set,1),1)],P1_set(:,3),-[P1_set(:,1:2),ones(size(P1_set,1),1)],zeros(size(P1_set,1),1));
            
            %% Distributing all points to farthest partition plane
            for i = 1:size(row_of_ind,1)
                point = [VehSpd(row_of_ind(i,1)), VehAccReq(row_of_ind(i,2)), VAFm(row_of_ind(i,1),row_of_ind(i,2))];
                val = zeros(K_act,1);
                for j = 1:K_act
                    val(j) = P_class(j).affine_eval(point(1:2));
                end
                [~,max_val_ind] = max(val);
                P_class(max_val_ind).P_set.val = [P_class(max_val_ind).P_set.val;point];
                P_class(max_val_ind).P_set.ind = [P_class(max_val_ind).P_set.ind;row_of_ind(i,:)];
            end
            
            %% Checking if new partitioning is same as previous and if so then terminate the iterations
            num = 0;
            for i = 1:K_act
                if(isequal(P_class(i).P_set.ind, P_class(i).P_set_pre.ind))
                    num = num + 1;
                end
            end
            
            if(num == K_act)
                break;
            end
            %% Removing partition if no point in it
            for  i = 1:K_act
                if(isempty(P_class(i).P_set))
                    P_class(i) = [];
                    K_act = K_act-1;
                end
            end
        end
        rms_sum = 0;
        for i = 1:K_act
            rms_sum = rms_sum + P_class(i).rms_error;
        end
        if(rms_sum < rms_N)
            rms_N = rms_sum;
            results(K).C = [];
            for i = 1:K_act
                results(K).C = [results(K).C;P_class(i).C];
            end
        end
    end
    results(K).rms_error = rms_N;
end