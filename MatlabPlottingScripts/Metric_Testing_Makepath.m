%housekeeping
clc
clear all
close all

%tuning parameters
% formula 1 waypoints
waypoints = [1,1.9;...
             2,1.9;...
             3.5,2.5];
% waypoints = [1.5,2.5;...
%              1.5,2.5];
             
% formula 2 waypoints
% waypoints = [2.5,2;...
%              0.5,0;...
%              2.5,-2];
% waypoints = [3,3.5;2,-5];

%formula 3 waypoints
% waypoints = [-2.5,2.5;0,0;0,2.5;0,0;2.5,2.5;0,0;0,0;0,0;0,0]; %this works
% waypoints = [-2.5,2.5;0,0;0,2.5;2.5,2.5];

%formula 5 waypoints
% waypoints = [1.5,3.5;2.5,4.5];

lqr_Q = 0.03*diag([100,1,100,1]);
lqr_R = 0.8*eye(2);

process_Q = [0.01, 0.001, 0.001, 0.001;...
             0.001, 0.01, 0.001, 0.001;...
             0.001, 0.001, 0.01, 0.001;...
             0.001, 0.001, 0.001, 0.01];
         
process_Q = 0.04*process_Q;

dt = 0.3; % discretization / "sampling" time (for applying process noise

x0 = [0.5;0.1;2.5;0];

cov_initial = zeros(4,4);

timetol = 2; %time allowed between waypoints

formula_3_manual = 0; %if using formula 3, will manually calculate max time away from charging station

%define linear system
A = [0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B = [0 0;1 0;0 0;0 1];
C = [1 0 0 0;0 0 1 0]; %different from anne's - we only care about position in specs
D = 0;

%find K, F using lqr 
K = lqr(A,B,lqr_Q,lqr_R);

F = inv(C*inv(-A+(B*K))*B);

%generate continous trajectory 
time_start = 0; %initialize integrator variables
globaltime = [];
globaltraj = [];
for i = 1:length(waypoints) % for each waypoint
    [localtime,localtraj] = ode45(@(t,x) EOM(t,x,A,B,K,F*waypoints(i,:)'),[time_start time_start+timetol],x0); %apply controller, get trajectory
    
    %append to global data
    globaltime = [globaltime;localtime]; 
    globaltraj = [globaltraj;localtraj];
    
    %update interator variables
    x0 = localtraj(end,:);
    time_start = time_start + timetol;
end

%find F for DT system (different from F gain used above)
F = expm(A*dt);

%discretize path, calculate noise, save data to matrix
time_last = 0;
cov_last = cov_initial;
data_matrix = [globaltime(1),globaltraj(1,:),cov_row_data(cov_last)];
figure(1)
hold on
grid on
for i = 1:length(globaltime)
    if globaltime(i) > (time_last +  dt)
        %calculate Covariance at this time step
        cov_last = F*cov_last*F' + process_Q;
        
        %save info to matrix
        data_matrix = [data_matrix;globaltime(i),globaltraj(i,:),cov_row_data(cov_last)];
        
        %update time
        time_last = globaltime(i);
        
        %plot point
        plot(globaltraj(i,1),globaltraj(i,3),'ko')
    end
end

plot(globaltraj(:,1),globaltraj(:,3))

%label plot
title('Plot Preview (verify trajectory speed/resolution is satisfactory)')
xlabel("Total time: " + num2str(globaltime(end,1)) + " seconds")

%write data and continous trajectory to csv
csvwrite('Matlab_path_data.csv',data_matrix)
csvwrite('Matlab_cont_path.csv',globaltraj)
csvwrite('Matlab_time_path.csv',globaltime)

%extra stuff
max_time_out_of_charge = 0;
in_flag = 1;
if formula_3_manual == 1
    for i = 1:length(data_matrix)
        if (data_matrix(i,2) < -0.5 || data_matrix(i,2) > 0.5 || data_matrix(i,4) > 0.5 || data_matrix(i,4) < -0.5) && in_flag == 1
            time_out = data_matrix(i,1);
            in_flag = 0;
        end
        if (data_matrix(i,2) > -0.5 && data_matrix(i,2) < 0.5 && data_matrix(i,4) < 0.5 && data_matrix(i,4) > -0.5) && in_flag == 0
            time_in = data_matrix(i,1);
            in_flag = 1;
            
            time_spent = time_in-time_out;
            if time_spent > max_time_out_of_charge
                max_time_out_of_charge = time_spent;
            end
        end
    end
end

max_time_out_of_charge
        

function myrow = cov_row_data(cov)
% save the top right of the covariance matrix as a row (only need half b/c
% diagonal
myrow = [cov(1,1),cov(1,2),cov(1,3),cov(1,4),cov(2,2),cov(2,3),cov(2,4),cov(3,3),cov(3,4),cov(4,4)];
end

function dx = EOM(t,x,A,B,K,FtimesRef)
u = -K*x + FtimesRef;
dx = A*x + B*u;
end
