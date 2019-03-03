% This script is used to do trajectory planning

%% Adaptively homotopic warm-starting approach (i.e. Algorithm 1 in the ICRA paper)
step = 0.2;                  % Initial setting of step
alpha = 0.5;                % Setting of \alpha_reduce
Nexpand = 10;            % Threshold of successive successful cycles that would make step increase 
epsilon_exit = 1e-5;     % Threshold to exit Algorithm 1 with a failure
epsilon_0 = 0.05;         % Gamma value related to Subproblem 0

%% Begin with \gamma = epsilon_0;
% Write geometric centers of all the obstacles in a file
GenerateGeometricCenters;
GenerateVertexes(epsilon_0);

%% Pre-solution with simple initialization
% Note that this small design is not mentioned in detail in our ICRA paper.
!ampl rf.run
!ampl r0.run
!ampl r1.run

%% Formally solve subproblem 0
!ampl r2.run
load flag.txt;

if (flag == 0) % If the simplest sub-problem fails, exit immediately.
    error 'Solution Failure at Subproblem 0.';
end

%% Initialization of the parameters.
gamma_achieved = epsilon_0;
counter = 0;

store_gamma = [];
store_step = [];

%% Sequential NLP-solving process
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
    GenerateVertexes(gamma_trial);
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
        error Intermediate_Subproblem_Solution_Failure_due_to_epsilon_exit_Criterion
    end
end

is_success = 1;