classdef ModelPredictiveController
    % MPC Controller
    properties
        model;
        % Initial system conditions
        tmeasure;       % time of measurement
        initial_state_real;     % initial state of the system
        measurement;    % robot state measurement
        measurements;

        % Output
        t;              % times
        state;          % robot states (x,y,theta,delta)
        input;          % input (v,delta_dot)
    end
    
    methods
        %% Constructor
        function controller = ModelPredictiveController()        
            controller.tmeasure =[];
            controller.initial_state_real=[0,0,0,0];
            controller.measurements= [];
            controller.t = [];
            controller.state = [];
            controller.input = [];
        end
 
        %% MPC parameters
        function [MPCIterations, p_lin, input0, state0, slip_ref] = init_iteration_params(t_ref, N, l, delta_t, m, n)
            
            % Number of iterations of solver 
            MPCIterations = length(t_ref) - N;
            % parameter : bicycle body length, sampling time
            p_lin = [l, delta_t];
        
            % Initiate input - (20x1)
            input0 = zeros(m*N, 1);
            % Initiate output -(40x1)
            state0 = zeros(n*N, 1);

            % Slip reference (2xn)
%             slip_ref = [zeros(2, floor(length(t_ref)/3)), 0.3*ones(2, ceil(2*length(t_ref)/3))];
            slip_ref = [zeros(2, length(t_ref))];

        end
        
    %% Stage cost
    function [H,f] = stage_cost(Q_mpc, R_mpc, N, n, m)
        Qstack = [];
        Rstack = [];
        for k = 1:N
            % Stage cost : weighting matrics for input (in discrete horizon) (40x40)
            Qstack = blkdiag(Qstack, Q_mpc);
            % Stage cost : weighting matrics for output (in discrete horizon) (20x20)
            Rstack = blkdiag(Rstack, R_mpc);
        end
        clear('k')
    
        % terminal cost = 0 (At end of horizon)
        Qstack = blkdiag(Qstack, zeros(n));
    
        % J(x, u_predict) = z'Hz 
        % J : Cost function
        % z : combining of predicted states and input
        % Stage cost : weighting matrics for output and input (in discrete horizon) (64x64)
        H = blkdiag(Qstack, Rstack);
        
        % system function (1x64)
        f = zeros(1, n*(N+1)+m*N);
    end

    %% Optimization
    % Mpc contraints
    function [Ai,bi, noise_mu, noise_cov,slip_estim] = mpc_constraint(delta_t, N, m, n, t_ref)
    
%         H_u = [ 1,  0, -1,  0;
%                -1,  0,  1,  0;
%                 0,  1,  0, -1;
%                 0, -1,  0,  1];
        
        S_u = [ 1,  0,  0,  0;
               -1,  0,  0,  0;
                0,  1,  0,  0;
                0, -1,  0,  0;
               -1,  0,  1,  0;
                1,  0, -1,  0;
                0, -1,  0,  1;
                0,  1,  0, -1];
        
        % velocity = acceleration * delta_t
        k_u = 4.0 * delta_t * ones(4,1);
        
        v_max = 0.7069;
        a_max = 44.3182;
        delta_max = pi/2;
        delta_dot_max = 3.2580;
    
        b_step_u = [v_max , -v_max, delta_max, -delta_max, a_max*delta_t, -a_max*delta_t, delta_dot_max*delta_t, -delta_dot_max*delta_t]'; 
        % Inequality constraints
        Au = [];
        bu = [];
    
        for k=0:N-2
            % time-variant dynamic matrix of system (36x20) ??????????   
            Au = [Au, zeros(size(Au,1), m);
                  zeros(size(S_u,1),k*m), S_u];
            % time-variant dynamic matrix of input (36x1) ???????????
            bu = [bu; b_step_u];
        end
        clear('k')
    
        %  inequality constraint matrix of robot system output 
        Ai = [zeros(size(Au,1),n*(N+1)), Au];
        %  inequality constraint matrix of control input 
        bi = bu;
    
        % Preallocation of variables (2x1189)
        slip_estim = zeros(2,length(t_ref)-N);
    
        % Set up random number generator for measurement noise
        rng('default');     % for reproducibility
        noise_mu = zeros(4,1);
        noise_sigma =  diag([1, 1, 2])*1e-4;
        noise_cov = chol(noise_sigma);
    end

    %% MPC Quadratic equality and inequality
    function [Aeq,beq,state_lin,input_lin] = mpc_equality_parameters(state_ref_mpc,input_ref_mpc,p_pred,N,n,m,state_estim)
        % predict horizon
        % Input:
        %   state_ref_mpc   :   state references of 1 iteration
        %   input_ref_mpc   :   input references of 1 iteration
        %   p_pred  :   mpc predict parameter
        %   N   :   number of steps in discrete horizon
        %   n   :   output dimensions (x,y,theta,δ)
        %   m   :   input dimensions (v,ω) 
        % Output:
        %   Aeq     :   equality constraints matrix of robot state
        %   beq     :   equality constraints matrix of input
        %   state_lin   :   Point on state ref to linearize around
        %   input_lin   :   Point on input ref to linearize around
        % Refernce :
        %   Daniel's thesis page 31
        
        % Build equality constraints (dynamics)
        % Subject to : Aeq*z = beq
        Aeq = zeros(n*(N+1), n*(N+1) + m*N);      % matrix of system (33x53)
        beq = zeros(n*(N+1), 1);                  % matrix of input (33x1)
        for k = 0:N-1
            % System matrices for system linearized around the reference
            state_lin = state_ref_mpc(:, 1+k); 
            input_lin = input_ref_mpc(:, 1+k); 
            %  Robot non linear system
            A_d = A_d_num(state_lin, input_lin, p_pred); 
            %  Robot non linear control input
            B_d = B_d_num(state_lin, input_lin, p_pred);
           
            % This part gets multiplied with the state part of the decision
            % variable, thus the Ad matrices
            % Aeq =[A_d -In B_d]
            % Aeq(1->30, 1->30)
            Aeq(n*k+1:n*(k+1), 1:n*(N+1)) = [zeros(n, n*k), A_d, -eye(n), zeros(n, n*(N-1-k))];
            % This part gets multiplied with the input part of the decision
            % variable, thus the Bd matrices
            % Aeq(1->30, 31->53)
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
        beq(n*N+1:n*(N+1))    = state_estim - state_lin;
        
        % Terminal constraints (Zero-terminal-constraint)
        % Careful: Problem might not be feasible for some initial values
        % Without feasibility no stability
        % Aeq(n*(N+1)+1:n*(N+2), :) = [zeros(n, n*N), eye(n), zeros(n, m*N)];
        % beq(n*(N+1)+1:n*(N+2))    = [0; 0; 0];
    end
    
    %% MPC Iterations
    function controller = iteration_(controller, t_ref, state_ref, input_ref) 
        
        % Robot parameters
        l = 0.53 / 2;   % wheel to center
        
        % System dimensions
        n = 4;   % state dimensions (x,y,theta,delta)
        m = 2;   % input dimensions (v,delta_dot)
        
        % Initial system conditions
        controller.tmeasure = 0.0;                                  % time of measurement
        controller.initial_state_real = [0; -0.1; pi/2];            % initial state of the system
        controller.measurement = controller.initial_state_real;     % robot state measurement
        controller.measurements = [];                               % robot states measurement
        
        % Set variables for output
        controller.t = [];        % times
        controller.state = [];    % robot states (x,y,theta,delta)
        controller.input = [];    % input (v,delta_dot)
    
        % MPC parameters
        delta_t = 0.05;     % Sampling time
        N = 10;             % discrete horizon
        T = N * delta_t;    % continuous horizon
        
        % Cost parameters
        Q_mpc = diag([50, 50, 3, 3]);    % weighting matrices for robot state > 0 (1x3)
        R_mpc = diag([0.1, 0.1]);        % weighting matrices for control input  > 0 (1x2)
    
        % Initiate parameters
        tol_opt       = 1e-8;
        options = optimset('Display','off',...
            'TolFun', tol_opt,...
            'MaxIter', 10000,...
            'TolConSQP', 1e-6);    
        
        [MPCIterations, p_lin, input0, state0, slip_ref] = init_iteration_params(t_ref, N, l, delta_t, m, n);    % MPCIterations, p_lin, input0, state0
        
        [H, f] = stage_cost(Q_mpc, R_mpc, N, n, m);
        
        [Ai, bi, noise_mu, noise_cov] = mpc_constraint(delta_t, N, m, n, t_ref);
        
        % Simulation from step 1 -> last Iteration
        for ii = 1:MPCIterations %(1->1189)
            % Measure the time
            t_Start = tic;
    
            % Get references for the current MPC loop
            % state references of 1 step
            state_ref_mpc = state_ref(:, ii:ii+N);
            % input references of 1 step
            input_ref_mpc = input_ref(:, ii:ii+N);
            
            % robot state estimation ( = measurement ) (x,y,theta)
            state_estim = controller.measurement;
            % ?????????????????
            p_pred = [p_lin, zeros(1,2)];
            
            % predict horizon
            [Aeq,beq,~,input_lin] = mpc_equality_parameters(state_ref_mpc, input_ref_mpc, p_pred, state_estim, N, n , m);
        
            % Solve optimization problem
            % Initial value for decision variable
            z0 = [state0; input0];
            [solutionOL,~,exit_flag,~] = quadprog(H, f, Ai, bi, Aeq, beq, [], [], z0, options);
            
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
            controller.t = [ controller.t, controller.tmeasure ];
            controller.state = [ controller.state, controller.initial_state_real ];
            controller.input = [ controller.input, input_wmr ];
            
            % NOTE: simulation
    
            % Update the closed-loop system
            % TODO: Make a function that using input_wmr or input to return initial_state_real
            % NOTE: kinematic
            p_real = [p_lin, slip_ref(:,ii)'];
            controller.initial_state_real = f_d_num_slip(controller.initial_state_real, input_wmr, p_real);
    
            % NOTE: simulate measurement
            measure_noise = randn(1,4)*noise_cov;
            measure_noise = noise_mu + measure_noise';
            controller.measurement = controller.initial_state_real + measure_noise;
            controller.tmeasure = controller.tmeasure + delta_t;
            
            controller.measurements = [ controller.measurements, controller.measurement ];
            
            % Prepare warmstart solution for next time step (take the endpiece of the optimal open-loop solution 
            % and add a last piece)
            state0 = [state_OL_tilde(n+1:end); state_estim - state_ref_mpc(:, N)];
            input0 = [input_OL_tilde(m+1:end); zeros(m, 1)];    
           
        end
    end



    end
end

