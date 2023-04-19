clear;
close all;
clc;

%% Execute code - Entry point

% initial setup
model = BicycleModel();

% get reference
trajectory = Trajectory();
trajectory = trajectory.read_ref("reference.csv");

%% Test : move robot along reference trajectory

x_res = [];
y_res = [];
theta_res = [];
delta_res = [];
model.delta = atan(2/10);

for i = 1:length(trajectory.t_ref)
    x_res = [x_res, model.x];
    y_res = [y_res, model.y];

%     if model.delta < atan(2/10)
%         model.step(1, 1.22)
%     elseif model.delta >= atan(2/10)
%         model.step(0, 0)
%     end
    model = model.step(1, 0);    
end

%% Plot
green = [0.0000, 0.6902, 0.3137];
grey = [0.2431,    0.2667,    0.2980];
f1 = figure(1);
f1.Color = 'w';
plot(x_res, y_res, 'Color', green, 'linewidth', 1.5), grid on, hold on
% plot(trajectory.x, trajectory.y, '--', 'Color', grey, 'linewidth', 1.5)
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
