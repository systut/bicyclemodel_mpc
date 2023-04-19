clear;
close all;
clc;

%% Execute code - Entry point

% initial setup
model = BicycleModel();

% get reference
trajectory = Trajectory();
trajectory = trajectory.read_ref("lemniscate_of_gerono_ref.csv");

% system controller

%% Test : move robot along reference trajectory

x_res = [];
y_res = [];
theta_res = [];
delta_res = [];
model.delta = trajectory.delta(1,1);
% controller.

% Trajectory + BicycleModel
for i = 1:length(trajectory.t_ref)
    x_res = [x_res, model.x];
    y_res = [y_res, model.y];
    theta_res = [theta_res, model.theta];
    delta_res = [delta_res, model.delta];
    model = model.step(trajectory.v(1,i), trajectory.delta_dot(1,i));    
end

% Trajectory + Controller + BicycleModel

% x_pred = [];
% y_pred = [];
% theta_pred = [];
% delta_pred = [];
% model.delta = trajectory.delta(1,1);
% controller.init_iteration_params(trajectory.t_ref, trajectory.state_ref, trajectory.input_ref)
% % Trajectory + BicycleModel
% for i = 1:length(trajectory.t_ref)
%     x_pred = [x_pred, model.x];
%     y_pred = [y_pred, model.y];
%     theta_pred = [theta_pred, model.theta];
%     delta_pred = [delta_pred, model.delta];
% %     controller = controller.iteration_(trajectory.t_ref, trajectory.state_ref, trajectory.input_ref);
%     
% end
%% Plot
green = [0.0000, 0.6902, 0.3137];
grey = [0.2431,    0.2667,    0.2980];
f1 = figure(1);
f1.Color = 'w';
plot(x_res, y_res, 'Color', green, 'linewidth', 1.5), grid on, hold on
plot(trajectory.x, trajectory.y, '--', 'Color', grey, 'linewidth', 1.5)
hold off;
box on;

% f2 = figure(2);
% f2.Color = 'w';
% plot(x_res, '-', 'Color', green, 'linewidth', 1.5), grid on, hold on
% plot(trajectory.x, '-', 'Color', grey, 'linewidth', 1.5)
% hold off;
% box on;
% 
% f3 = figure(3);
% f3.Color = 'w';
% plot(y_res, '-', 'Color', green, 'linewidth', 1.5), grid on, hold on
% plot(trajectory.y, '-', 'Color', grey, 'linewidth', 1.5)
% hold off;
% box on;
% 
% f4 = figure(4);
% f4.Color = 'w';
% plot(theta_res, '-', 'Color', green, 'linewidth', 1.5), grid on, hold on
% plot(trajectory.theta, '-', 'Color', grey, 'linewidth', 1.5)
% hold off;
% box on;
