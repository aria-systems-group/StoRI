clc
clear all 
close all

formula = 2;
% load tree
treedat = load("tree_data.csv");

figure(1)
hold on
for i = 1:length(treedat)
    plot([treedat(i,1),treedat(i,3)],[treedat(i,2),treedat(i,4)],'b');
    plot(treedat(i,1),treedat(i,2),'bo')
end

% plot spec
if formula == 1
    av_x = [1,2,2,1,1];
    av_re_y = [2,2,3,3,2];
    re_x = [3,4,4,3,3];

    plot(av_x,av_re_y,'r')
    plot(re_x,av_re_y,'g')
    
    formulastr = "'G[0,2]!A & F[0,18]B'  with Interval [";
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

if formula == 5
    go_x = [2,3,3,2,2];
    go_y = [4,4,5,5,4];
    
    pud_x = [2,3,3,2,2];
    pud_y = [2,2,3,3,2];
    
    carp_x = [1,2,2,1,1];
    carp_y = [3,3,4,4,3];
    
    plot(go_x,go_y,'g')
    plot(pud_x,pud_y,'b')
    plot(carp_x,carp_y,'b--')
end

axis equal
xlim([-5 5])
ylim([-5 5])
grid on


