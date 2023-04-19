%% generate reference trajectory for wmr
% Needs MATLAB 2019a starting in line 133 (because of writecell).
% But you can just use the already generated reverence.csv for the other
% files. Another way is to use csvwrite, but then you cannot include
% headers for the columns (and you have to change all the files using
% reference.csv, because I removed the header line there)
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear 
close all
clc

T_max = 60;
t = linspace(0,T_max,20*T_max);     % 1200 steps -  should take 60s to complete with 20 Hz sampling rate
delta_t = 0.05;                     % sampling time
L = 2;                              % distance between the front and rear axles
R = 10;                              % 'radius'
w = 2*pi/T_max;                     % angular velocity
dir = 1;                            % direction of traversion, only +/- 1 possilbe

% Lemniscate of gerono with constant angular velocity w
x_d = R*sin(2*w*t) / 2;
y_d = R*(cos(w*t)-1);
% First derivative of adapted lemniscate of gerono
x_d_dot = R*w*cos(2*w*t);
y_d_dot = -R*w*sin(w*t);
% Second derivative of adapted lemniscate of gerono
x_d_d_dot = -2*R*w*w*sin(2*w*t);
y_d_d_dot = -R*w*w*cos(w*t);

% Reference for v_d
v_d = sqrt(x_d_dot.^2 + y_d_dot.^2);
% Reference for δ_d (steering angle)
delta_d = atan(L * (y_d_d_dot .* x_d_dot - x_d_d_dot .* y_d_dot) ./ (v_d.^3));
% Reference for δ_dot_d (steering rate)
delta_d_dot = zeros(1,length(t));
for i = 1:length(t)-1
    delta_d_dot(1,i) = (delta_d(1,i+1) - delta_d(1,i)) / delta_t; 
end

% Reference for theta_d and theta_d_dot (yaw velocity)
theta_d = atan2(y_d_dot, x_d_dot);
% theta_d = atan(y_d_dot / x_d_dot);

% Save as timeseries to use as input
v_ts = timeseries(v_d, t);
delta_dot_ts = timeseries(delta_d_dot, t);

%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% plot the reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];
yellow = [1.0000, 0.8353, 0.0000];


% xy-plot
f1 = figure(1);
f1.Color = 'w';
plot(x_d, y_d, '-', 'Color', blue, 'linewidth', 1.5);
grid on;
box on;
xlim([-0.6*R,0.6*R])
ylim([-2.5*R,0.5*R])

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');

% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/xy_reference.tex','width','\fwidth', 'encoding', 'utf8')


% theta and theta_dot plot
f2 = figure(2);
f2.Color = 'w';
hold on
plot(t, delta_d, '-', 'Color', yellow, 'linewidth', 1.5);
plot(t, theta_d, '-', 'Color', blue, 'linewidth', 1.5);
% plot(t, theta_d_dot, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max])
ylim([-2.5,2.5])

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('$\theta$ in $\mathrm{rad}$, $\dot{\theta}$ in $\mathrm{rad/s}$',...
        'interpreter', 'latex');
legend({'$\theta$', '$\dot{\theta}$'}, 'interpreter', 'latex',...
                                                            'orientation', 'vertical',...
                                                            'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/theta_reference.tex','width','\fwidth', 'encoding', 'utf8')


% input (v_d, delta_d) plot
f3 = figure(3);
f3.Color = 'w';
hold on
plot(t, v_d, '-', 'Color', blue, 'linewidth', 1.5);
plot(t, delta_d_dot, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max]);
ylim([-3, 3]);

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('wheel velocities in $\mathrm{m/s}$', 'interpreter', 'latex');
legend({'$v_{d}$', '$delta dot_{d}$'}, 'interpreter', 'latex',...
                                               'orientation', 'vertical',...
                                               'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
% matlab2tikz('figures/input_reference.tex','width','\fwidth', 'encoding', 'utf8')


%% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
ref = [t', x_d', y_d', theta_d', delta_d', v_d', delta_d_dot'];
% round reference to reduce file size
ref = round(ref, 7);
% header line to explain columns
header = {'Time', 'x_d', 'y_d', 'theta_d', 'delta_d','v_d', 'delta_d_dot'};
output = [header; num2cell(ref)];
writecell(output, 'reference.csv'); % introduced in Matlab 2019a

