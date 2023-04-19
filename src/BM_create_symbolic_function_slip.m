%% simulate and linearize ODEs using the symbolic toolbox
clear
close all
clc

if ~exist('BM_functions', 'dir')
  mkdir('BM_functions')
end
addpath('BM_functions')

%% Init nonlinear ODE & linearization

% ----- Define symbolic variables -----
% Robot states(output)
    x = sym('x',[4,1]);     %[x, y, theta, delta]
% Robot inputs
    u = sym('u',[2,1]);     %[v, delta_dot];

    syms L delta_t
    syms i_r i_l
    parameters = [L, delta_t, i_r,i_l]; % wheel to center, sample time

% linear state and input
    x_lin = sym('x_lin',[4,1]);
    u_lin = sym('u_lin',[2,1]);
% non-linear state and input
    x_k   = sym('x_k',[4,1]);
    u_k   = sym('u_k',[2,1]);

    f = [u(1)*cos(x(3));        % v * cos(theta)
         u(1)*sin(x(3));        % v * sin(theta)
         u(1)*tan(x(4))/L;      % v * tan(delta) / L
         u(2)];                 % delta dot

% linearized discrete time system
    A_lin   = subs(jacobian(f,x),[x;u],[x_lin;u_lin]);
    B_lin   = subs(jacobian(f,u),[x;u],[x_lin;u_lin]);
    
    A_d     = eye(length(x)) + delta_t*A_lin;
    B_d     = delta_t * B_lin;
    
% nonlinear discrete time system
    f_d     = x_k + delta_t*subs(f,[x;u],[x_k;u_k]);

% ----- Export matlab functions -----
    matlabFunction(f,  'File','BM_functions/f_num_slip'  ,'Vars',{x,u,parameters});
    matlabFunction(f_d,'File','BM_functions/f_d_num_slip','Vars',{x_k,u_k,parameters});
    matlabFunction(A_d,'File','BM_functions/A_d_num_slip','Vars',{x_lin,u_lin,parameters});
    matlabFunction(B_d,'File','BM_functions/B_d_num_slip','Vars',{x_lin,u_lin,parameters});
%% Simulate reference trajectory

%----- load trajectory -----
    ref   = reference();
    ref   = ref';
    t_ref = ref(1,:);
    x_ref = ref(2:5,:);
    u_ref = ref(6:7,:);

% ----- parameters -----
    L = 2;                % Front axis to back axis
    delta_t = 0.05;       % Sampling time
    i_r = 0.3;
    i_l = 0.3;
    param = [L, delta_t i_r, i_l];     % Parameters

% ----- try different points of linearization -----
    idx_lin = 1;
    % idx_lin = length(t_ref)*1/4;  
    % idx_lin = length(t_ref)*1/2;  
    % idx_lin = length(t_ref)*3/4;  

% ----- Simulation -----
    x_sim      = 0*x_ref;               % get vector with same length as x_ref
    x_sim(:,1) = x_ref(:,1);            % initial state of simulation
    
    % x_lin      = x_ref(:,idx_lin);    % point around which system will be linearized
    % u_lin      = u_ref(:,idx_lin);
    
    %linearization (fixed point)
    % A = A_d_num(x_lin,u_lin,p);
    % B = B_d_num(x_lin,u_lin,p);
         
    % dx = x_sim - x_lin;
    % du = u_ref - u_lin;

    x_sim_lin = 0*x_ref;
    x_sim_lin(:,1) = x_ref(:,1);
    
% ----- simulate nonlinear & linear discrete time system -----
    for k = 1:length(t_ref)-1
        x_lin = x_ref(:, k);
        u_lin = u_ref(:, k);
        
        A = A_d_num_slip(x_lin,u_lin,param);
        B = B_d_num_slip(x_lin,u_lin,param);
        
        dx = x_sim(:,k) - x_lin;
        du = u_ref(:,k) - u_lin;
        
        x_sim(:,k+1) = f_d_num(x_sim(:,k),u_ref(:,k),param);
        dx    = A*dx + B*du;
        
        x_sim_lin(:,k+1) = dx + x_lin;
    end

% add linearization point
% x_sim_lin = dx + repmat(x_lin,[1,length(dx)]);

% ----- plot -----
    fig = figure(31);
    fig.Color = 'w';
    
    cla;
    hold on;
    plot(x_sim(1,:),x_sim(2,:),'linewidth',2)
    plot(x_sim_lin(1,:),x_sim_lin(2,:),'linewidth',2)
    plot(x_ref(1,:),x_ref(2,:),'k--','linewidth',2)
    grid on;
    box on;
    hold off;
    xlim([-0.6,0.6])
    ylim([-2.5,0.5])
    ax  = gca();
    ax.TickLabelInterpreter = 'latex';
    xlabel('position $x_1(t)$ in $\mathrm{m}$','interpreter','latex')
    ylabel('position $x_2(t)$ in $\mathrm{m}$','interpreter','latex')
    legend({'nonlinear ODE','linear ODE','reference'},'interpreter','latex',...
                                                      'orientation','horizontal',...
                                                      'location','south')





%% reference trajectory

function ref = reference()
    t = linspace(0,60,20*60); % should take 60s to complete with 20 Hz sampling rate
    L = 2;   % wheel to center
    delta_t = 0.05; 
    w = (2*pi)/60;

    % Lemniscate of gerono, adapted so that one period takes 60s
    x_d = sin(2*w*t) / 2;
    y_d = cos(w*t)-1;
    % First derivative of adapted lemniscate of gerono
    x_d_dot = w*cos(2*w*t);
    y_d_dot = -w*sin(w*t);
    % Second derivative of adapted lemniscate of gerono
    x_d_ddot = -2*w*w*sin(2*w*t);
    y_d_ddot = -w*w*cos(w*t);

    v = sqrt(x_d_dot.^2 + y_d_dot.^2);
    % Reference for theta_ and theta_dot, calculated from the previous
    % references and the system model without any slip
    theta_d = atan2(y_d_dot, x_d_dot);
    delta_d = atan(L * (y_d_ddot .* x_d_dot - x_d_ddot .* y_d_dot) ./ (sqrt(x_d_dot.^2 + y_d_dot.^2) .^3));

    for i = 1:length(t)-1
        delta_d_k_1 = delta_d(1,i+1);
        delta_d_k = delta_d(1,i);
        delta_d_dot(1,i) = (delta_d_k_1 - delta_d_k) / delta_t; 
    end
    delta_d_dot(1, length(t)) = 0;
    % Save as timeseries to use as input for dynamics.slx
%     v_ts = timeseries(v, t);
%     delta_d_dot_ts = timeseries(delta_d_dot, t);

    % Plot the references for x, y and theta
    figure
    plot(x_d, y_d);
    figure
    plot(t, v);
    hold on
    plot(t, delta_d_dot);
    grid on

    ref = [t', x_d', y_d', theta_d', delta_d', v', delta_d_dot'];
end