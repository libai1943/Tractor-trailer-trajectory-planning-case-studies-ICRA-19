% ==============================================================================
% MATLAB Source Codes for "Trajectory Planning for a Tractor with Multiple
% Trailers in Extremely Narrow Environments: A Unified Approach".

% ==============================================================================

%   Copyright (C) 2019 Bai Li
%   User must cite the following article if they utilize these codes for
%   new publications: Bai Li et al., "Trajectory Planning for a Tractor
%   with Multiple Trailers in Extremely Narrow Environments: A Unified
%   Approach", To appear in IEEE 2019 International Conference on Robotics
%   and Automation (ICRA).

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
global polygon_obstacle_vertex
%%%%%%%%  Experiment setup %%%%%%%%

% Case 1

%% Initial configuration %%
x0_1 = -10;         % x_1(t = 0)
y0_1 = -17;         % y_1(t = 0)
theta0_1 = 0;      % theta_1(t = 0)
theta0_2 = 0;      % theta_2(t = 0)
theta0_3 = 0;      % theta_3(t = 0)
theta0_4 = 0;      % theta_4(t = 0)
phy_0 = 0;          % phy(t = 0)
v_0 = 0;              % v(t = 0)
a_0 = 0;              % a(t = 0)
w_0 = 0;             % w(t = 0)

%% Terminal configuration %%
% We create a box with the geometric center being (x_center_tf, y_center_tf)
% The box is 16 m length and 2.6 m width
x_center_tf = 12;       % geometric center along x axis
y_center_tf = 0;         % geometric center along y axis
v_tf = 0;                    % v(t = tf)
a_tf = 0;                    % a(t = tf)
w_tf = 0;                   % w(t = tf)

%% Obstacle setup %%
% Vertex points of N_obs obstable are record in a 1 x (8*N_obs) vector. For example,
% suppose we have one rectangular obstacle with vertexes V1 (v1x, v2x), V2
% (v2x, v2y), V3(v3x, v3y), and V4(v4x, v4y), then the vector should be
% [v1x, v1y, v2x, v2y, v3x, v3y, v4x, v4y].
polygon_obstacle_vertex = [-20 -15 -10 -15 -5 0 -20 5 0 -20 20 -20 20 -2 -2 -2];
boundary_constraints = [x0_1, y0_1, theta0_1, theta0_2, theta0_3, theta0_4, phy_0, v_0, a_0, w_0, x_center_tf, y_center_tf, v_tf, a_tf, w_tf];

%% Other alternative cases %%
% % Case 2
% polygon_obstacle_vertex = [-12.81,2.42,-11.56,1.01,-15.19,-2.2,-16.44,-0.79,-5.85,5.12,-4.38,5.99,-2.05,2.08,-3.52,1.21,0.47,-2.18,0.38,-4.06,-4.47,-3.82,-4.38,-1.94, -10,5-8,-8,7-7.5,-5,-5-8,-10,-4-8];
% boundary_constraints = [-10,10,0,0,0,0,0,0,0,0,0,0,0,0,0];
%
% % Case 3
% polygon_obstacle_vertex = [0, 1.8, 10, 2, 10, 20, 0, 20, 10, 2, 10, 20, 20, 20, 20, 1, 0, -1.8, 20, -1.8, 20, -30, 0, -30];
% boundary_constraints = [-3,-10,pi/2,pi/2,pi/2,pi/2,0,0,0,0,8,0,0,0,0];

%% Prepare for Trajectory planning %%
PrepareTrajectoryPlanning;


%% Trajectory planning %%
AdaptivelyHomotopicWarmStartingApproach;


%% Solution illustration
if (is_success)
    figure (101)
    plot(store_gamma);
    xlabel('Number of Cycle');
    ylabel('\gamma_a_c_h_i_e_v_e_d')
    title('Evolution of \gamma_a_c_h_i_e_v_e_d');
    
    figure (102)
    plot(store_step);
    xlabel('Number of Cycle');
    ylabel('step')
    title('Evolution of step');
    
    figure (103)
    DrawTrajectories; % Plot the optimized trajectories.
    
    %%% If reader has interest, the dynamic process can be observed through
    %%% the following function.
    % VideoGeneration
end
