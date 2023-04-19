% This script implements linear mpc for a bicycle robot.
clear;
close all;
clc
if ~exist('BM_functions', 'dir')
  mkdir('BM_functions')
end
addpath('BM_functions')

%% Execution - Entry point
[t_ref, state_ref, input_ref, x_ref, y_ref, theta_ref, delta_ref, v_ref, delta_dot_ref, slip_ref] =  get_reference("reference.csv");

[t,input,state,tmeasure,measurements] = execute_mpc_iteration(t_ref, state_ref, input_ref, slip_ref, delta_ref);

plot_result(x_ref, y_ref, t, measurements, state, input, state_ref)
%% Function

function [t_ref, state_ref, input_ref, x_ref, y_ref, theta_ref, delta_ref, v_ref, delta_dot_ref, slip_ref] =  get_reference(file_name)

    % ----- Read reference file ------
        ref = csvread(file_name,2,0);

        ref = ref';

    % ----- Get parameters ------
        t_ref = ref(1,:);           % Time

        % ----- State (Output) Reference ------
        x_ref = ref(2,:);           % coordinate x
        
        y_ref = ref(3,:);           % Coordinate y
        
        theta_ref = ref(4,:);       % Orientation 
        
        delta_ref = ref(5,:);       % Steering angle

        state_ref = [x_ref; y_ref; theta_ref; delta_ref];
    % ----- Input reference ------
        v_ref = ref(6,:);           % Input velocity 
        
        delta_dot_ref = ref(7,:);   % Input steering rate
        
        input_ref = [v_ref; delta_dot_ref];
    
     % ----- Slip reference -----
        slip_ref = [zeros(2, length(t_ref))];
end

function [t,input,state,tmeasure,measurements] = execute_mpc_iteration(t_ref, state_ref, input_ref, slip_ref, delta_ref)
    
    % ----- Settings -----
        tol_opt       = 1e-8;
        
        options = optimset('Display','off',...
            'TolFun', tol_opt,...
            'MaxIter', 10000,...
            'TolConSQP', 1e-6);

    % ----- Execute functions  -----
        % Parameters
        [N, m, n, mpc_iteration, param, ~, delta_t] = generate_mpc_parameters(t_ref);
        
        % Initial
        [tmeasure, state_real, measurement, input0, state0, measurements, t, state, input] = initiate_robot_state(N, m, n, delta_ref);
        
        % Cost function
        [H, f] = generate_state_cost_matrix(N, m, n);
        
        % Constrain
        [Ai, Bi] = generate_inequality_constraint(delta_t, N, m, n);
        
        % Noise and future error        
        [~, noise_mu, noise_cov] = generate_noise_and_slip(t_ref, N);
        
    % ----- Output (Control signal)  -----
        for ii = 1:mpc_iteration
           % Measure the time
                t_Start = tic;

           % state references of 1 step
                state_ref_mpc = state_ref(:, ii:ii+N);

            % input references of 1 step
                input_ref_mpc = input_ref(:, ii:ii+N);
            
            % robot state estimation ( = measurement ) (x,y,θ)
                state_estim = measurement;
           
                param_pred = [param, zeros(1,2)];
            
            % predict horizon
                [Aeq, Beq, ~, input_lin] = generate_equality_constraint(N, m, n, state_ref_mpc, input_ref_mpc, param_pred, state_estim);
      
            % Solve optimization problem
            % Initial value for decision variable
                z0 = [state0; input0];

                [solutionOL,~,exit_flag,~] = quadprog(H, f, Ai, Bi, Aeq, Beq, [], [], z0, options);
            
            if exit_flag ~= 1
                exit_flag;
            end
            
            % Derive optimal predicted state and input sequence
                state_OL_tilde = solutionOL(1:n*(N+1), 1);
                input_OL_tilde = solutionOL(n*(N+1)+1:end, 1);
                state_OL = reshape(state_OL_tilde, n, N+1);
                input_OL = reshape(input_OL_tilde, m, N);
            
            % Get control input needed for the simulated WMR
                input_wmr = input_OL(:,1) + input_lin;
            
            % Stop Timing
                t_Elapsed = toc( t_Start );
            
            % Store closed-loop data;
                t = [ t, tmeasure ];
                state = [ state, state_real ];
                input = [ input, input_wmr ];
            
            % NOTE: simulation
    
            % Update the closed-loop system
            % TODO: Make a function that using input_wmr or input to return state_real
                p_real = [param, slip_ref(:,ii)'];
                state_real = f_d_num(state_real, input_wmr, p_real);
    
            % NOTE: simulate measurement
                measure_noise = randn(1,4)*noise_cov;
                measure_noise = noise_mu + measure_noise';
                measurement = state_real + measure_noise;
                tmeasure = tmeasure + delta_t;
            
                measurements = [ measurements, measurement ];
            
            % Prepare warmstart solution for next time step (take the endpiece of the optimal open-loop solution 
            % and add a last piece)
                state0 = [state_OL_tilde(n+1:end); state_estim - state_ref_mpc(:, N)];
                input0 = [input_OL_tilde(m+1:end); zeros(m, 1)];                   
        end
end

function [N, m, n, mpc_iteration, param, T, delta_t] = generate_mpc_parameters(t_ref)
    
    % ----- Fixed parameters ------
        n = 4;          % Input dimension (x, y, theta, delta)
        m = 2;          % Output dimension (v, delta_dot)
        l = 1;          % Distance of front and rear axis
        mpc_iteration = length(t_ref) - 10;   % Number of MPC times will be executed
    % ----- Changable parameters ------
        N = 10;         % Number of MPC step in predicted horizon
        delta_t = 0.05;   % Sampling time
    %--------------------------------------------------------------------------
        T = N*delta_t;                      % Time per predicted horizon
        param = [l, delta_t];               
end

function [tmeasure, state_real, measurement, input0, state0, measurements, t, state, input] = initiate_robot_state(N, m, n,delta_ref)
     % ----- Initial system conditions -----
        tmeasure = 0.0;                   % time of measurement
        state_real = [0; 0; 0; 0];        % initial state of the system
        measurement = state_real;         % robot state measurement
        input0 = zeros(m*N, 1);           % Initial input in predicted horizon
        state0 = zeros(n*N, 1);           % Initial output in predicted horizon
    
    % ----- Robot states measurement -----
        measurements = [];

    % ----- Set variables for output -----
        t = [];        % times
        state = [];    % robot states
        input = [];    % input 
end

function [H, f] = generate_state_cost_matrix(N, m, n)
    % This function considers about cost function for Optimization Problem

    % ----- Weighting parameters ------
        Q_mpc = diag([1, 1, 1, 0.1]);     % Weighting matrix for state (output)      
        R_mpc = diag([2, 0.1]);             % Weighting matrix for input
        
    % ----- Changable parameters ------
        Qstack = [];                
        Rstack = [];
        for k = 1:N
            % Stage cost : weighting matrixs for input (in discrete horizon) (40x40)
            Qstack = blkdiag(Qstack, Q_mpc);
            % Stage cost : weighting matrixs for output (in discrete horizon) (20x20)
            Rstack = blkdiag(Rstack, R_mpc);
        end
        clear('k')

    % ---------------------------------------
        H = blkdiag(zeros(n), Qstack, Rstack);
        % system cost function (1x64)
        f = zeros(1, n*(N+1)+m*N);
end

function [Ai, Bi] = generate_inequality_constraint(delta_t, N, m, n)
    % ----- Changable parameters ------
        % limitation for input 
        v_max =  0.5;
        v_min = -0.5;
        a_max =  0.35;
        a_min = -0.35;
        delta_max = pi/4;
        delta_min = -pi/4;
        delta_dot_max = pi/2;
        delta_dot_min = pi/2;
        
        S_u = [ 1,  0, -1,  0;
               -1,  0,  1,  0;
                0,  1,  0, -1;
                0, -1,  0,  1];
        B_step_u = [a_max*delta_t, a_min*delta_t, delta_dot_max*delta_t, delta_dot_min*delta_t]';

        % state and input's inequality constraints
%         S_u = [ 1,  0,  0,  0;      % v(k) <= v_max
%                -1,  0,  0,  0;      % v(k) >= v_min 
%                 0,  1,  0,  0;      % delta(k) <= delta_max
%                 0, -1,  0,  0;      % delta(k) >= delta_min
%                -1,  0,  1,  0;      % v(k+1) - v(k) <= a_max * delta_t
%                 1,  0, -1,  0;      % v(k+1) - v(k) >= a_min * delta_t
%                 0, -1,  0,  1;      % delta(k+1) - delta(k) <=  delta_dot_max * delta_t
%                 0,  1,  0, -1];     % delta(k+1) - delta(k) >=  delta_dot_min * delta_t
% 
%         B_step_u = [v_max , v_min, delta_max, delta_min, a_max*delta_t, a_min*delta_t, delta_dot_max*delta_t, delta_dot_min*delta_t]'; 
    % ----- Inequality constraints ------
        Au = [];
        Bu = [];
    
        for k=0:N-2
            % time-variant dynamic matrix of system (36x20) ??????????   
            Au = [Au, zeros(size(Au,1), m);
                  zeros(size(S_u,1),k*m), S_u];
            % time-variant dynamic matrix of input (36x1) ???????????
            Bu = [Bu; B_step_u];
        end
        clear('k')
    
        %  inequality constraint matrix of robot system output 
        Ai = [zeros(size(Au,1),n*(N+1)), Au];
        %  inequality constraint matrix of control input 
        Bi = Bu;
end

function [Aeq, Beq, state_lin, input_lin] = generate_equality_constraint(N, m, n, state_ref_mpc, input_ref_mpc, param_pred, state_estim)

    % -------- Input --------:
        %   state_ref_mpc   :   state references of 1 iteration
        %   input_ref_mpc   :   input references of 1 iteration
        %   param_pred  :   mpc predict parameter
        %   N   :   number of steps in discrete horizon
        %   n   :   output dimensions (x, y , theta, delta)
        %   m   :   input dimensions (v, delta_dot) 
    % -------- Output --------:
        %   Aeq     :   equality constraints matrix of robot state
        %   beq     :   equality constraints matrix of input
        %   state_lin
        %   input_lin
    % -------- Reference -------- :
        %   Daniel's thesis page 31
        %   Build equality constraints (dynamics)
        %   Subject to : Aeq*z = beq
        
    % -------- Function ----------
        Aeq = zeros(n*(N+1), n*(N+1) + m*N);
        Beq = zeros(n*(N+1), 1); 
        for k = 0:N-1
            %  ????????????????? System matrices for system linearized around the reference
                state_lin = state_ref_mpc(:, 1+k);
                input_lin = input_ref_mpc(:, 1+k);
            %  ????????????????? Robot non linear system
                A_d = A_d_num_slip(state_lin, input_lin, param_pred);
            %  ????????????????? Robot non linear control input
                B_d = B_d_num_slip(state_lin, input_lin, param_pred);

            % This part gets multiplied with the state part of the decision variable, thus the Ad matrices
                Aeq(n*k+1:n*(k+1), 1:n*(N+1)) = [zeros(n, n*k), A_d, -eye(n), zeros(n, n*(N-1-k))];
            % This part gets multiplied with the input part of the decision variable, thus the Bd matrices
                Aeq(n*k+1:n*(k+1), n*(N+1)+1:end) = [zeros(n, m*k), B_d, zeros(n, m*(N-1-k))];

        end
        clear('k')
            
        % Overwrite state_lin and input_lin from the previous loop to set it to
        % values for the initial timestep of the prediction
        state_lin = state_ref_mpc(:, 1);
        input_lin = input_ref_mpc(:, 1);
        
        % Equality constraints (initial constraint)
        % Aeq(31->33, all)
        Aeq(n*N+1:n*(N+1), :) = [eye(n), zeros(n, n*N+m*N)];
        % beq(31->33)
        Beq(n*N+1:n*(N+1))    = state_estim - state_lin;
        
        % Terminal constraints (Zero-terminal-constraint)
        % Careful: Problem might not be feasible for some initial values
        % Without feasibility no stability
        % Aeq(n*(N+1)+1:n*(N+2), :) = [zeros(n, n*N), eye(n), zeros(n, m*N)];
        % beq(n*(N+1)+1:n*(N+2))    = [0; 0; 0];
end

function [slip_estim, noise_mu, noise_cov] = generate_noise_and_slip(t_ref, N)

        % ----- Pre-allocation of variables -----
            slip_estim = zeros(2,length(t_ref)-N);
        
        % ----- Set up random number generator for measurement noise -----
            rng('default');      % for reproducibility
            noise_mu = zeros(4,1);
            noise_sigma =  diag([1, 1, 1, 1])*1e-4;
            noise_cov = chol(noise_sigma);
end

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Plot results
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

function plot_result(x_ref, y_ref, t, measurements, state, input, state_ref)

    % Corporate design colors
    grey = [0.2431, 0.2667,0.2980];
    red = [0.7529, 0.0000, 0.000];
    green = [0.0000, 0.6902, 0.3137];
    blue = [0.0000, 0.3176, 0.6196];
    yellow = [1.0000, 0.8353, 0.0000];
    black = [0, 0, 0];
    
    % Position in x-y-plane
%     f1 = figure(1);
%     f1.Color = 'w';
%     plot(x_ref, y_ref, '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on
%     plot(measurements(1,:), measurements(2,:), '--', 'Color', green, 'linewidth', 1.5)
%     plot(state(1,:), state(2,:), '-', 'Color', blue, 'linewidth', 1.5)
%     hold off;
%     box on;
%     ylim([-3.0,0.5]);

    % Position in x-y-plane
%     f1 = figure(1);
%     hold on
%     f1.Color = 'w';
%     plot(t(1,:), input(1,:), 'Color', blue, 'linewidth', 1.5)
%     hold off;
%     grid on;
%     box on;
%     
%     ax = gca();
%     ax.TickLabelInterpreter = 'latex';
%     xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
%     ylabel('velocities in $\mathrm{m/s}$', 'interpreter', 'latex');
%     legend({'$v_{d}$'}, 'interpreter', 'latex',...
%                        'orientation', 'vertical',...
%                        'location', 'southeast');
%     f2 = figure(2);
%     hold on
%     f2.Color = 'w';
%     plot(t(1,:), input(2,:), 'Color', blue, 'linewidth', 1.5)
%     hold off;
%     grid on;
%     box on;
% 
%     ax = gca();
%     ax.TickLabelInterpreter = 'latex';
%     xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
%     ylabel('steering rates in $\mathrm{m/s}$', 'interpreter', 'latex');
%     legend({'$\omega_{d}$'}, 'interpreter', 'latex',...
%                        'orientation', 'vertical',...
%                        'location', 'southeast');

    for i= 1:length(state)
        error_x(1,i) = state(1,i) - state_ref(1,i); 
        error_y(1,i) = state(2,i) - state_ref(2,i); 
    end
    f3 = figure(3);
    f3.Color = 'w';
    hold on;
    plot(t(1,:), error_x(1,:), 'Color', black, 'linewidth', 1.5)
    hold off;
    grid on;
    box on;

    ax = gca();
    ax.TickLabelInterpreter = 'latex';
    xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
    ylabel('error in x', 'interpreter', 'latex');
    legend({'error in x'}, 'interpreter', 'latex',...
                       'orientation', 'vertical',...
                       'location', 'southeast');

    f4 = figure(4);
    f4.Color = 'w';
    hold on;
    plot(t(1,:), error_y(1,:), 'Color', black, 'linewidth', 1.5)
    hold off;
    grid on;
    box on;

    ax = gca();
    ax.TickLabelInterpreter = 'latex';
    xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
    ylabel('error in y', 'interpreter', 'latex');
    legend({'error in y'}, 'interpreter', 'latex',...
                       'orientation', 'vertical',...
                       'location', 'southeast');
end


