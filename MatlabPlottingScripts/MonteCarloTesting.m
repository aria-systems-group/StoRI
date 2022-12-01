% This script will (eventually) perform montecarlo simulations of resulting
% controllers to measure statistical probability of success MUST RUN
% INITBREACH FROM BREACH DIRECTORY FIRST!
clc
clear all
close all

formula = 1;
%% User Inputs
dt = 0.25; %dt to create discrete system
if formula == 1
   x0 = [0.5;0.1;2.5;0];
   STL_ReadFile('form1.stl');
elseif formula == 2
    x0 = [0.5;0.1;1.5;0];
    STL_ReadFile('form2.stl');
elseif formula == 3
    x0 = [1.5;0.1;1;0.1];
    STL_ReadFile('form3.stl');
end

Q = [0.01 0.001 0.001 0.001;...
     0.001 0.01 0.001 0.001;...
     0.001 0.001 0.01 0.001;...
     0.001 0.001 0.001 0.01];
Q = 0.001*Q;

startcov = zeros(4,4);


Rphi = BreachRequirement(phi);

numsims = 1000;
%% load csv, extract controls
path_data = load("control_path.csv");
interval_data = load("Interval.csv");

controls = path_data(:,15:16);
times = path_data(:,18);
states = path_data(:,1:4);

%% Define Linearized System
A = [0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B = [0 0;1 0;0 0;0 1];

%% Define DT System
Ahat = [A,B;zeros(2,6)];
expmahat = expm(Ahat*dt);

F = expmahat(1:4,1:4);
G = expmahat(1:4,5:6);

%%% MONTE CARLO STUFF!
successes = 0;
for i = 1:numsims
    %% Generate Discret Trajectory
    x0_sim = mvnrnd(x0,startcov)'; %add noise to starting state
    path = x0_sim';
    curtime = 0;
    timepath = 0;

    while curtime < times(end)
        i = find(times > curtime,1);
        u = controls(i,:)';
        xnext = F*x0_sim + G*u + mvnrnd([0;0;0;0],Q)'; %add noise at every step
        curtime = curtime + dt;
        path = [path;xnext'];
        timepath = [timepath;curtime];
        x0_sim = xnext;
    end
    
    %%Interpolate
    testtimepath = 0:0.05:timepath(end);
    testpath1 = interp1(timepath,path(:,1),testtimepath);
    testpath3 = interp1(timepath,path(:,3),testtimepath);
%     plot(path(:,1),path(:,3),'bo')
%     hold on
%     plot(testpath1,testpath3,'r*')
%     close
    

    %% Make Trace
    Bdata = BreachTraceSystem({'x', 'y'});
    trace = [testtimepath' testpath1' testpath3'];
    Bdata.AddTrace(trace);

    win = Rphi.Eval(Bdata);
    
    if win > 0
        successes = successes + 1;
    end
    
end
disp("Lower Bound of StoRI: " + num2str(interval_data(1)))
disp("Success Rate from Monte Carlo Sims: " + num2str(successes/numsims))

function dx = EOM(t,x,A,B,K,FtimesRef)
u = -K*x + FtimesRef;
dx = A*x + B*u;
end