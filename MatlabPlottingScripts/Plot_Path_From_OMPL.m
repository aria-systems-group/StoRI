clc
clear all
close all

global p2 p3

formula = 3;
dt = 0.15;
plot_error = true; % set true to plot covariance ellipses
error_bound = 0.9; % percent bound desired for covariance ellipses

% load csv
path_data = load("../build/control_path.csv");
interval_data = load("../build/Interval.csv");

% set initial state (nonlinearized)
x0lin = path_data(1,1:4)';
theta0 = atan2(x0lin(4),x0lin(2));
v0 = sqrt(x0lin(2)^2 + x0lin(4)^2);
x0 = [x0lin(1);x0lin(3);theta0;v0];

% integrate (nonlinear) dynamics using control inputs given by data
time1 = 0;
timeonline = 0;
finaltime = path_data(end,18);
global_time = [];
global_data = [];
data_size = size(path_data);
i = 2;
while time1 < finaltime-0.0001
    if time1 >= path_data(i,18)-0.0001
        i = i + 1;
    end
    u = path_data(i,15:16);
    time2 = time1 + dt;
    [local_time,local_data] = ode45(@(t,x) nonlinearEOM(t,x,u),[time1 time2],x0);
    
    global_time = [global_time;local_time];
    global_data = [global_data;local_data];
    x0 = local_data(end,:);
    time1 = time1 + dt;
end

% plot
figure(1)
hold on

% plot spec
if formula == 1
    av_x = [1,2,2,1,1];
    av_re_y = [2,2,3,3,2];
    re_x = [3,4,4,3,3];
    
    wall_x = [0,4,4,0,0];
    wall_y = [0,0,3,3,0];

    patch(av_x, av_re_y, 'black', 'FaceColor', 'red', 'FaceAlpha', 0.7);
    patch(re_x, av_re_y, 'black', 'FaceColor', 'green', 'FaceAlpha', 0.7);
    plot(wall_x,wall_y,'r')
    
    formulastr = "Trajectory for \phi_1  with Interval [";
end
if formula == 2
    av_x = [1.5,3.5,3.5,1.5,1.5];
    av_y = [-0.5,-0.5,0.5,0.5,-0.5];
    
    go1_x = [2,3,3,2,2];
    go1_y = [1,1,2,2,1];
    
    go2_x = [2,3,3,2,2];
    go2_y = [-1,-1,-2,-2,-1];
    
    wall_x = [0,4,4,0,0];
    wall_y = [-2,-2,2,2,-2];
    
    figure(1)
    set(gcf,'Position',[200 200 800 700])
    
    patch(av_x, av_y, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.7);
    patch(go1_x, go1_y, 'black', 'FaceColor', '#F2DEA2', 'FaceAlpha', 1);
    patch(go2_x, go2_y, 'black', 'FaceColor', '#F2DEA2', 'FaceAlpha', 1);
    plot(wall_x,wall_y,'k')
    
    ax = gca;
    ax.FontSize = 20;
    
    formulastr = "Trajectory for \phi_2 with Interval [";
end

if formula == 3
    go_x = [2,3,3,2,2];
    go_y = [4,4,5,5,4];
    
    pud_x = [0,2.5,2.5,0,0];
    pud_y = [2,2,3,3,2];
    
    carp_x = [0,1,1,0,0];
    carp_y = [4,4,5,5,4];
    
    wall_x = [0,3,3,0,0];
    wall_y = [0,0,5,5,0];
    
    figure(1)
    set(gcf,'Position',[200 200 500 500])
    
    patch(go_x, go_y, 'black', 'FaceColor', '#F2DEA2', 'FaceAlpha', 1);
    patch(pud_x,pud_y, 'black', 'FaceColor', '#71B1D9', 'FaceAlpha', 1);
    patch(carp_x,carp_y, 'black','LineStyle','--', 'FaceColor', 'black', 'FaceAlpha', 0.3);
    plot(wall_x,wall_y,'k')
    set(gca,'xdir','Reverse')
    camroll(-90)
    
    ax = gca;
    ax.FontSize = 20;
    ax.YAxisLocation = 'Right';
    
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    
    formulastr = "Trajectory for \phi_3 with" + newline + "interval [";
    
end

% plot error ellipses
if plot_error
    covdat = load('../build/cov_dat.csv');
    data_size = size(covdat);
    for i = 1:data_size(1)
        cov4 = buildCov4x4(covdat(i,5:14));%extract 4x4 covariance
        cov2 = [cov4(1,1),cov4(1,3);cov4(1,3),cov4(3,3)];%extract 2x2 covariance
        mean = [covdat(i,1),covdat(i,3)];%extract mean
        plotErrorEllipse(mean, cov2, error_bound);%plot the error ellipse    
    end
end

% plot trajectory
p1 = plot(global_data(:,1),global_data(:,2),'k'); % trajectory

title(formulastr + num2str(interval_data(1)) + ", " + num2str(interval_data(2)) + "]")
axis image

function dx = nonlinearEOM(t,x,ulin)
%calculate actual nonlinear controls
vdot = ulin(1)*cos(x(3)) + ulin(2)*sin(x(3));
omega = ((-ulin(1)*sin(x(3)))+(ulin(2)*cos(x(3))))/x(4);

%implement nonlinear dynamics
dx = [(x(4)*cos(x(3)));(x(4)*sin(x(3)));omega;vdot];
end

function plotErrorEllipse(mu, Sigma, p)
    global p2
    
    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    p2 = patch(a(1, :) + mu(1), a(2, :) + mu(2), 'black', 'FaceColor', '#AED8F2', 'FaceAlpha', 0.7);
end

function plotErrorEllipseAlt(mu, Sigma, p)
    global p3
    
    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    p3 = patch(a(1, :) + mu(1), a(2, :) + mu(2), 'black', 'FaceColor', '#F2CDC4', 'FaceAlpha', 0.7);
end

function cov = buildCov4x4(values)
cov = [values(1),values(2),values(3),values(4);...
       values(2),values(5),values(6),values(7);...
       values(3),values(6),values(8),values(9);...
       values(4),values(7),values(9),values(10)];
end