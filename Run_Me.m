% ==============================================================================
% MATLAB Source Codes for "Trajectory Planning for a Tractor with Multiple
% Trailers in Extremely Narrow Environments: A Unified Approach".   

% ==============================================================================

%   Copyright (C) 2018-2019 Bai Li
%   Users must cite the following article if they utilize these codes for
%   new publications:
%   Bai Li et al., "Trajectory Planning for a Tractor with Multiple
%   Trailers in Extremely Narrow Environments: A Unified Approach",
%   Accepted in ICRA 2019.  

% ==============================================================================

% If there are inquiries, feel free to contact libai@zju.edu.cn
%
% (Very Important) The AMPL utilized in this pack is just a TRIAL version.
% The users must delete AMPL.exe in this version after trying it, and
% then apply for their own valid license files through following the
% official instructions at https://ampl.com/try-ampl/request-a-full-trial/

% ==============================================================================
clear
clc
close

tic
% Obstacle setup
global Polygon_obstacle_vertex

% % Case 1
% Polygon_obstacle_vertex = [-20 -15 -10 -15 -5 0 -20 5 0 -20 20 -20 20 -2 -2 -2];
% Two_point_boundary_constraint([-10,-17,0,0,0,0,0,0,0,0], [12,0,0,0,0,0,0,0,0,0]);

% % Case 2
% Polygon_obstacle_vertex = [-12.81,2.42,-11.56,1.01,-15.19,-2.2,-16.44,-0.79,-5.85,5.12,-4.38,5.99,-2.05,2.08,-3.52,1.21,0.47,-2.18,0.38,-4.06,-4.47,-3.82,-4.38,-1.94, -10,5-8,-8,7-7.5,-5,-5-8,-10,-4-8];
% Two_point_boundary_constraint([-10,10,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0]);

 % Case 3
Polygon_obstacle_vertex = [0, 1.8, 10, 2, 10, 20, 0, 20, 10, 2, 10, 20, 20, 20, 20, 1, 0, -1.8, 20, -1.8, 20, -30, 0, -30];
Two_point_boundary_constraint([-3,-10,pi/2,pi/2,pi/2,pi/2,0,0,0,0], [8,0,0,0,0,0,0,0,0,0]);

NE = 80;
NCNC = 4;

step = 0.2; % Initial setting of step
alpha = 0.5; % Setting of \alpha_reduce
Nexpand = 10; % Threshold of successive successful cycles that would make step increase 
epsilon_exit = 1e-5;
epsilon_0 = 0.05;

% Begin with \gamma = epsilon_0;
Center_generation;
Vertex_generation(epsilon_0);

!ampl rf.run
!ampl r0.run
!ampl r1.run
!ampl r2.run

load flag.txt;
if (flag == 0) % If the simplest sub-problem fails, exit immediately.
    error 'Solution Failure at Subproblem 0.';
end

% Initial settings of the parameters. The names are in accordance with the
% parameters in the manuscript.

gamma_achieved = epsilon_0;
gamma_trial = 0;
counter = 0;

store_gamma = [];
store_tf = [];
store_step = [];

% The following while loops never terminates until the originla problem
% (i.e. the one with \gamma_achieved == 1) is solved successfully OR
% epsilon_exit related threshold is activated.
while (gamma_achieved ~= 1)
    store_step = [store_step, step]; % Record the current step value
    if ((gamma_achieved + step) <= 1) % Check if the next gamma_trial would exceed the upper bound
        gamma_trial = gamma_achieved + step;
    else % If it exceeds, it would be set to 1
        gamma_trial = 1;
    end
    Vertex_generation(gamma_trial);
    store_gamma = [store_gamma, gamma_achieved]; % Record the current successful gamma_achieved
    
    !ampl r2.run
    load flag.txt; % Check if the current optimization trail succeeds: 1 = succeeds, 0 = fails.
    
    if (flag == 1) % If succeeds
        gamma_achieved = gamma_trial; % Update the current successful gamma_trail as gamma.
        counter = counter + 1; % Increase the counter by 1.
    else % If fails
        step = step .* alpha; % Reduce "step" because a failure happens
        counter = 0; % Reset the counter
    end
    if (counter >= Nexpand)
        step = step ./ alpha;
        counter = 0; 
    end
    fclose('all');
    
    if (step <= epsilon_exit)
        error "Intermediate_Subproblem_Solution_Failure"
    end
end

toc

figure (1)
plot(store_gamma);
xlabel('Number of Cycle');
ylabel('\gamma_a_c_h_i_e_v_e_d')
title('Evolution of \gamma_a_c_h_i_e_v_e_d');

figure (2)
draw_trajectories(NE, NCNC); % Plot the optimized trajectories.