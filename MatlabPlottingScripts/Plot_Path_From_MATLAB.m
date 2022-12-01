clc
clear all
close all

plot_error = true;
title_flag = false;
formula = 2;

% load csvs
cont_path = load("Matlab_cont_path.csv");
path_data = load("Matlab_path_data.csv");
time_data = load("Matlab_time_path.csv");
interval_data = load("Interval.csv");

data_size = size(path_data);


% % parse trajectory for split in time (6 seconds)
% time_6_ind = find(global_time > 6,1);
% if(isempty(time_6_ind))
%     time_6_ind = length(global_time);
% end

hold on
% plot spec
if formula == 1
    av_x = [1,2,2,1,1];
    av_re_y = [2,2,3,3,2];
    re_x = [3,4,4,3,3];
    
    wall_x = [0,4,4,0,0];
    wall_y = [0,0,3,3,0];

    figure(1)
    set(gcf,'Position',[300 300 800 400])
    patch(av_x, av_re_y, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.3);
    patch(re_x, av_re_y, 'black', 'FaceColor', '#F2DEA2', 'FaceAlpha', 1);
%     plot(wall_x,wall_y,'r')
    
    formulastr = "Trajectory with StoRI [";
    
    ax = gca;
    ax.FontSize = 16;
    xlabel('X','FontSize',16, 'Interpreter','Latex')
    ylabel('Y','FontSize',16, 'Interpreter','Latex')
%     plot(0.5,2.5,'p','Markersize',18)
%     text(0.32,2.65,'Start','FontSize',24)
%     text(1.2,2.5,'Obstacle','FontSize',24)
%     text(3.35,2.5,'Goal','FontSize',24)
    ylim([1 3])
    xlim([0 4])
%     axis equal
%     axis image
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
    
    patch(av_x, av_y, 'black', 'FaceColor', 'black', 'FaceAlpha', 0.7);
    patch(go1_x, go1_y, 'black', 'FaceColor', 'green', 'FaceAlpha', 0.7);
    patch(go2_x, go2_y, 'black', 'FaceColor', 'green', 'FaceAlpha', 0.7);
    plot(wall_x,wall_y,'r')
    
    formulastr = "Trajectory for \phi_2 with Interval [";
end

if formula == 3
    av_x = [1,4,4,1,1];
    av_y = [1,1,3,3,1];
    
    go1_x = [0,1,1,0,0];
    go1_y = [1,1,2,2,1];
    
    go2_x = [4,5,5,4,4];
    go2_y = [2,2,3,3,2];
    
    wall_x = [0,5,5,0,0];
    wall_y = [0,0,3,3,0];
    
    plot(av_x,av_y,'r--')
    plot(go1_x,go1_y,'g')
    plot(go2_x,go2_y,'g')
    plot(wall_x,wall_y,'r')
    
    formulastr = "Trajectory for \phi_2 with Interval [";
end

if formula == 4
    go_x = [2,3,3,2,2];
    go_y = [4,4,5,5,4];
    
    pud_x = [0,2.5,2.5,0,0];
    pud_y = [2,2,3,3,2];
    
    carp_x = [0,1,1,0,0];
    carp_y = [4,4,5,5,4];
    
    wall_x = [0,3,3,0,0];
    wall_y = [0,0,5,5,0];
    
    plot(go_x,go_y,'g')
    plot(pud_x,pud_y,'b')
    plot(carp_x,carp_y,'b--')
    plot(wall_x,wall_y,'r')
    
    formulastr = "Trajectory for \phi_5 with interval [";
end

global p2

if plot_error
    for i = 1:data_size(1)
        cov4 = buildCov4x4(path_data(i,6:15));%extract 4x4 covariance
        cov2 = [cov4(1,1),cov4(1,3);cov4(1,3),cov4(3,3)];%extract 2x2 covariance
        mean = [path_data(i,2),path_data(i,4)];%extract mean
        plotErrorEllipse(mean, cov2, 0.9);%plot the error ellipse    
    end
end

% plot
figure(1)
% p1 = patch([cont_path(:,1)' nan],[cont_path(:,3)' nan],[time_data' nan],'EdgeColor','Interp','LineWidth',2);
p1 = plot(cont_path(:,1),cont_path(:,3),'k');

% legend([p1 p2],'Expected Trajectory','90% Confidence Ellipses','Location','northoutside','FontSize',16)

xarrow = [0.41,0.5];
yarrow = [2.65,2.5];
[xact,yact] = ds2nfu(xarrow,yarrow);
mystr = "$$t = 0,\quad $$" +  newline + "$$\tilde{f} = [0,1]$$";
annotation('textarrow',xact,yact,'string',mystr,'FontSize',26,'Interpreter','Latex')

xarrow = [0.75,0.9746];
yarrow = [1.5,1.9462];
[xact,yact] = ds2nfu(xarrow,yarrow);
annotation('textarrow',xact,yact,'string','$$t = 2, \tilde{f} = [0,1]$$','FontSize',26, 'Interpreter','Latex')

xarrow = [2,1.9352];
yarrow = [1.25,1.8837];
[xact,yact] = ds2nfu(xarrow,yarrow);
annotation('textarrow',xact,yact,'string','$$t = 4, \tilde{f} = [0,0.847]$$','FontSize',26, 'Interpreter','Latex')

xarrow = [3.41,3.41];
yarrow = [1.5,2.46];
[xact,yact] = ds2nfu(xarrow,yarrow);
annotation('textarrow',xact,yact,'string','$$t = 6, \tilde{f} = [0.35,0.847]$$','FontSize',26, 'Interpreter','Latex')
% colorbar
% colormap(turbo)
%plot(path_data(:,2),path_data(:,4),'bo') 

%legend('Path in First 6 Seconds', 'Path After 6 Seconds', 'Region A', 'Region B','Covariance Ellipses')
if title_flag
    title(formulastr + num2str(interval_data(1)) + ", " + num2str(interval_data(2)) + "]",'FontSize',16)
end



function plotErrorEllipse(mu, Sigma, p)
    global p2
    
    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    p2 = patch(a(1, :) + mu(1), a(2, :) + mu(2), 'black', 'FaceColor', '#F2CDC4', 'FaceAlpha', 0.3);
%     plot(a(1, :) + mu(1), a(2, :) + mu(2),'b');
end

function cov = buildCov4x4(values)
cov = [values(1),values(2),values(3),values(4);...
       values(2),values(5),values(6),values(7);...
       values(3),values(6),values(8),values(9);...
       values(4),values(7),values(9),values(10)];
end